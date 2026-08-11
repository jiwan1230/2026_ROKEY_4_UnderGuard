"""쥐 감지 기록·로봇 이동 경로를 SQLite에 영구 저장한다.

실시간 뷰(StateManager, CameraFrameStore)와 의도적으로 분리한다 — 그쪽은
"지금 이 순간" 상태만 메모리에 들고 있다가 서버가 꺼지면 사라지는 게 원칙이고,
이 저장소는 정반대로 "사용자가 자리를 비운 사이 무슨 일이 있었는지" 나중에
조회하기 위한 기록용이다. 실시간 폴링(/api/snapshot)에는 관여하지 않는다.

일반 기록 API는 조회 전용으로 유지한다. 다만 사용자가 명시적으로 확인한 경우에
한해 로컬 기록 전체를 비우는 관리 작업을 지원한다. 이 저장소는 운영 MySQL과
분리돼 있으므로 해당 작업이 Robot DB 기록에는 영향을 주지 않는다.
"""

from __future__ import annotations

import sqlite3
import threading
import time
from pathlib import Path
from typing import Any


class HistoryStore:
    """탐지 기록과 이동 경로 점을 저장·조회하는 SQLite 래퍼."""

    def __init__(self, db_path: str | Path, image_dir: str | Path) -> None:
        self.db_path = Path(db_path)
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self.image_dir = Path(image_dir)
        self.image_dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        # 콜백/요청이 여러 스레드에서 올 수 있어 커넥션 소유 검사를 끄고 락으로 직접 보호한다.
        self._conn = sqlite3.connect(str(self.db_path), check_same_thread=False)
        self._init_schema()

    def _init_schema(self) -> None:
        with self._lock:
            self._conn.execute(
                """CREATE TABLE IF NOT EXISTS detections (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp REAL NOT NULL,
                    robot_id TEXT,
                    object_type TEXT,
                    map_x REAL,
                    map_y REAL,
                    confidence REAL,
                    image_path TEXT,
                    opening_id TEXT,
                    trap_id TEXT,
                    trap_installation_status TEXT,
                    is_dummy INTEGER NOT NULL DEFAULT 0
                )"""
            )
            # 기존 history.db를 지우지 않고 새 필드를 추가한다. 운영 MySQL의
            # opening/trap 조회 API가 연결되기 전까지 이 필드는 UI 계약과 더미
            # 데이터 확인에 사용하며, API 연결 후에도 같은 응답 이름을 유지한다.
            detection_columns = {
                row[1]
                for row in self._conn.execute("PRAGMA table_info(detections)")
            }
            for column, definition in (
                ("opening_id", "TEXT"),
                ("trap_id", "TEXT"),
                ("trap_installation_status", "TEXT"),
            ):
                if column not in detection_columns:
                    self._conn.execute(
                        "ALTER TABLE detections ADD COLUMN "
                        f"{column} {definition}"
                    )
            self._conn.execute(
                """CREATE TABLE IF NOT EXISTS trail_points (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp REAL NOT NULL,
                    robot_id TEXT NOT NULL,
                    map_x REAL,
                    map_y REAL,
                    is_dummy INTEGER NOT NULL DEFAULT 0
                )"""
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_detections_ts ON detections(timestamp)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_trail_robot_ts ON trail_points(robot_id, timestamp)"
            )
            self._conn.commit()

    def record_detection(
        self,
        *,
        robot_id: str | None,
        object_type: str | None,
        map_x: float | None,
        map_y: float | None,
        confidence: float | None = None,
        opening_id: str | None = None,
        trap_id: str | None = None,
        trap_installation_status: str | None = None,
        timestamp: float | None = None,
        image_bytes: bytes | None = None,
        image_ext: str = "jpg",
        is_dummy: bool = False,
    ) -> int:
        """탐지 기록 한 건을 저장한다. 이미지가 있으면 파일로도 함께 저장한다.

        입력: 로봇·객체 종류·좌표 등 탐지 정보와, 있다면 원본 이미지 바이트다.
        출력: 새로 생긴 기록의 id다(이미지 조회 URL을 만들 때 쓴다).
        사용: 실제 로봇 감지가 배선되면 ros_bridge에서, 지금은 더미 시드
        스크립트(seed_dummy_history.py)에서 호출한다.
        """

        ts = time.time() if timestamp is None else timestamp
        is_opening = object_type == "ENTRY_POINT"
        if is_opening:
            normalized_trap_status = (
                trap_installation_status or "UNKNOWN"
            ).strip().upper()
            if normalized_trap_status not in {
                "INSTALLED",
                "NOT_INSTALLED",
                "UNKNOWN",
            }:
                raise ValueError("지원하지 않는 트랩 설치 상태입니다.")
        else:
            opening_id = None
            trap_id = None
            normalized_trap_status = None
        with self._lock:
            cur = self._conn.execute(
                "INSERT INTO detections "
                "(timestamp, robot_id, object_type, map_x, map_y, confidence, "
                "image_path, "
                "opening_id, trap_id, trap_installation_status, is_dummy) "
                "VALUES (?,?,?,?,?,?,?,?,?,?,?)",
                (
                    ts,
                    robot_id,
                    object_type,
                    map_x,
                    map_y,
                    confidence,
                    None,
                    opening_id,
                    trap_id,
                    normalized_trap_status,
                    int(is_dummy),
                ),
            )
            detection_id = cur.lastrowid
            if image_bytes is not None:
                filename = f"{detection_id}.{image_ext}"
                (self.image_dir / filename).write_bytes(image_bytes)
                self._conn.execute(
                    "UPDATE detections SET image_path=? WHERE id=?",
                    (filename, detection_id),
                )
            self._conn.commit()
            return detection_id

    def record_trail_point(
        self,
        *,
        robot_id: str,
        map_x: float,
        map_y: float,
        timestamp: float | None = None,
        is_dummy: bool = False,
    ) -> None:
        """로봇 위치 한 점을 이동 경로 기록에 추가한다."""

        ts = time.time() if timestamp is None else timestamp
        with self._lock:
            self._conn.execute(
                "INSERT INTO trail_points (timestamp, robot_id, map_x, map_y, is_dummy) "
                "VALUES (?,?,?,?,?)",
                (ts, robot_id, map_x, map_y, int(is_dummy)),
            )
            self._conn.commit()

    def list_detections(
        self,
        *,
        limit: int = 200,
        since: float | None = None,
        until: float | None = None,
        object_type: str | None = None,
        robot_id: str | None = None,
        trap_installation_status: str | None = None,
    ) -> list[dict[str, Any]]:
        """최신순으로 탐지 기록을 반환한다. 필터는 전부 선택 사항이다."""

        query = (
            "SELECT id, timestamp, robot_id, object_type, map_x, map_y, "
            "confidence, image_path, opening_id, trap_id, "
            "trap_installation_status, is_dummy FROM detections"
        )
        clauses: list[str] = []
        params: list[Any] = []
        if since is not None:
            clauses.append("timestamp >= ?")
            params.append(since)
        if until is not None:
            clauses.append("timestamp <= ?")
            params.append(until)
        if object_type is not None:
            clauses.append("object_type = ?")
            params.append(object_type)
        if robot_id is not None:
            clauses.append("robot_id = ?")
            params.append(robot_id)
        if trap_installation_status is not None:
            if trap_installation_status == "NOT_INSTALLED_OR_UNKNOWN":
                clauses.append(
                    "object_type = 'ENTRY_POINT' AND "
                    "COALESCE(trap_installation_status, 'UNKNOWN') "
                    "IN ('NOT_INSTALLED', 'UNKNOWN')"
                )
            else:
                clauses.append(
                    "object_type = 'ENTRY_POINT' AND "
                    "COALESCE(trap_installation_status, 'UNKNOWN') = ?"
                )
                params.append(trap_installation_status)
        if clauses:
            query += " WHERE " + " AND ".join(clauses)
        query += " ORDER BY timestamp DESC LIMIT ?"
        params.append(limit)

        with self._lock:
            rows = self._conn.execute(query, params).fetchall()
        return [
            {
                "id": row[0],
                "timestamp": row[1],
                "robot_id": row[2],
                "object_type": row[3],
                "map_x": row[4],
                "map_y": row[5],
                "confidence": row[6],
                "image_url": f"/api/history/detections/{row[0]}/image" if row[7] else None,
                "opening_id": row[8],
                "trap_id": row[9],
                "trap_installation_status": (
                    row[10] or "UNKNOWN"
                    if row[3] == "ENTRY_POINT"
                    else None
                ),
                "is_dummy": bool(row[11]),
            }
            for row in rows
        ]

    def get_trail(
        self,
        *,
        robot_id: str | None = None,
        since: float | None = None,
        until: float | None = None,
        limit: int = 5000,
    ) -> list[dict[str, Any]]:
        """오래된 것부터 최신 순으로 경로 점을 반환한다(선으로 이어 그리기 좋게)."""

        query = "SELECT id, timestamp, robot_id, map_x, map_y FROM trail_points"
        clauses: list[str] = []
        params: list[Any] = []
        if robot_id is not None:
            clauses.append("robot_id = ?")
            params.append(robot_id)
        if since is not None:
            clauses.append("timestamp >= ?")
            params.append(since)
        if until is not None:
            clauses.append("timestamp <= ?")
            params.append(until)
        if clauses:
            query += " WHERE " + " AND ".join(clauses)
        query += " ORDER BY timestamp ASC LIMIT ?"
        params.append(limit)

        with self._lock:
            rows = self._conn.execute(query, params).fetchall()
        return [
            {"id": row[0], "timestamp": row[1], "robot_id": row[2], "map_x": row[3], "map_y": row[4]}
            for row in rows
        ]

    def image_path_for(self, detection_id: int) -> Path | None:
        """탐지 id에 연결된 이미지 파일 경로다. 없으면 None이다."""

        with self._lock:
            row = self._conn.execute(
                "SELECT image_path FROM detections WHERE id=?", (detection_id,)
            ).fetchone()
        if not row or not row[0]:
            return None
        path = self.image_dir / row[0]
        return path if path.is_file() else None

    def delete_dummy_records(self) -> dict[str, int]:
        """시연용 더미 행과 그 증거 이미지만 제거한다.

        웹 API에는 연결하지 않고 ``seed_dummy_history.py --replace-dummy``에서만
        사용한다. 실제 ROS 기록(``is_dummy=0``)은 건드리지 않는다.
        """

        with self._lock:
            image_names = [
                row[0]
                for row in self._conn.execute(
                    "SELECT image_path FROM detections "
                    "WHERE is_dummy=1 AND image_path IS NOT NULL"
                ).fetchall()
            ]
            detection_count = self._conn.execute(
                "SELECT COUNT(*) FROM detections WHERE is_dummy=1"
            ).fetchone()[0]
            trail_count = self._conn.execute(
                "SELECT COUNT(*) FROM trail_points WHERE is_dummy=1"
            ).fetchone()[0]
            self._conn.execute("DELETE FROM detections WHERE is_dummy=1")
            self._conn.execute("DELETE FROM trail_points WHERE is_dummy=1")
            self._conn.commit()

        removed_images = 0
        for image_name in image_names:
            # record_detection()이 만든 basename만 허용해 오래된 DB 값이
            # 이미지 폴더 밖의 파일을 가리키더라도 삭제하지 않는다.
            if Path(image_name).name != image_name:
                continue
            image_path = self.image_dir / image_name
            if image_path.is_file():
                image_path.unlink()
                removed_images += 1
        return {
            "detections": int(detection_count),
            "trail_points": int(trail_count),
            "images": removed_images,
        }

    def clear_records(self) -> dict[str, int]:
        """로컬 탐지·이동 기록 전체와 연결된 증거 이미지를 제거한다.

        운영 MySQL이나 Replay JSON에는 접근하지 않는다. 이미지 삭제는 DB에
        저장된 안전한 basename만 대상으로 하고, 기록과 동시에 들어오는 ROS
        콜백과 충돌하지 않도록 전체 작업 동안 같은 저장소 락을 유지한다.
        """

        with self._lock:
            image_names = [
                row[0]
                for row in self._conn.execute(
                    "SELECT image_path FROM detections "
                    "WHERE image_path IS NOT NULL"
                ).fetchall()
            ]
            detection_count = self._conn.execute(
                "SELECT COUNT(*) FROM detections"
            ).fetchone()[0]
            trail_count = self._conn.execute(
                "SELECT COUNT(*) FROM trail_points"
            ).fetchone()[0]
            self._conn.execute("DELETE FROM detections")
            self._conn.execute("DELETE FROM trail_points")
            self._conn.commit()

            removed_images = 0
            for image_name in image_names:
                if Path(image_name).name != image_name:
                    continue
                image_path = self.image_dir / image_name
                if image_path.is_file():
                    image_path.unlink()
                    removed_images += 1

        return {
            "detections": int(detection_count),
            "trail_points": int(trail_count),
            "images": removed_images,
        }

    def summary(
        self,
        *,
        since: float | None = None,
        until: float | None = None,
        object_type: str | None = None,
        robot_id: str | None = None,
        trap_installation_status: str | None = None,
    ) -> dict[str, int]:
        """현재 기록 화면 필터에 해당하는 탐지·경로 건수를 반환한다."""

        detection_clauses: list[str] = []
        detection_params: list[Any] = []
        trail_clauses: list[str] = []
        trail_params: list[Any] = []

        for clause, value in (
            ("timestamp >= ?", since),
            ("timestamp <= ?", until),
            ("robot_id = ?", robot_id),
        ):
            if value is None:
                continue
            detection_clauses.append(clause)
            detection_params.append(value)
            trail_clauses.append(clause)
            trail_params.append(value)
        if object_type is not None:
            detection_clauses.append("object_type = ?")
            detection_params.append(object_type)
        if trap_installation_status is not None:
            if trap_installation_status == "NOT_INSTALLED_OR_UNKNOWN":
                detection_clauses.append(
                    "object_type = 'ENTRY_POINT' AND "
                    "COALESCE(trap_installation_status, 'UNKNOWN') "
                    "IN ('NOT_INSTALLED', 'UNKNOWN')"
                )
            else:
                detection_clauses.append(
                    "object_type = 'ENTRY_POINT' AND "
                    "COALESCE(trap_installation_status, 'UNKNOWN') = ?"
                )
                detection_params.append(trap_installation_status)

        detection_query = "SELECT COUNT(*) FROM detections"
        trail_query = "SELECT COUNT(*) FROM trail_points"
        if detection_clauses:
            detection_query += " WHERE " + " AND ".join(detection_clauses)
        if trail_clauses:
            trail_query += " WHERE " + " AND ".join(trail_clauses)

        with self._lock:
            detections = self._conn.execute(
                detection_query, detection_params
            ).fetchone()[0]
            trail_points = self._conn.execute(
                trail_query, trail_params
            ).fetchone()[0]
        return {"detections": detections, "trail_points": trail_points}
