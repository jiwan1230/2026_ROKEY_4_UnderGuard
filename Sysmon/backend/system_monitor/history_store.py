"""쥐 감지 기록·로봇 이동 경로를 SQLite에 영구 저장한다.

실시간 뷰(StateManager, CameraFrameStore)와 의도적으로 분리한다 — 그쪽은
"지금 이 순간" 상태만 메모리에 들고 있다가 서버가 꺼지면 사라지는 게 원칙이고,
이 저장소는 정반대로 "사용자가 자리를 비운 사이 무슨 일이 있었는지" 나중에
조회하기 위한 기록용이다. 실시간 폴링(/api/snapshot)에는 관여하지 않는다.

조회 전용(read-only) API만 노출한다 — 감시 대시보드의 "기록을 남기고 볼 수는
있지만 지우거나 고칠 수는 없다"는 원칙을 이 계층에도 그대로 유지한다.
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
                    is_dummy INTEGER NOT NULL DEFAULT 0
                )"""
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
        with self._lock:
            cur = self._conn.execute(
                "INSERT INTO detections "
                "(timestamp, robot_id, object_type, map_x, map_y, confidence, image_path, is_dummy) "
                "VALUES (?,?,?,?,?,?,?,?)",
                (ts, robot_id, object_type, map_x, map_y, confidence, None, int(is_dummy)),
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
    ) -> list[dict[str, Any]]:
        """최신순으로 탐지 기록을 반환한다. 필터는 전부 선택 사항이다."""

        query = (
            "SELECT id, timestamp, robot_id, object_type, map_x, map_y, "
            "confidence, image_path, is_dummy FROM detections"
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
                "is_dummy": bool(row[8]),
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

    def summary(self) -> dict[str, int]:
        """조회 탭 헤더 등에 쓸 전체 건수 요약이다."""

        with self._lock:
            detections = self._conn.execute("SELECT COUNT(*) FROM detections").fetchone()[0]
            trail_points = self._conn.execute("SELECT COUNT(*) FROM trail_points").fetchone()[0]
        return {"detections": detections, "trail_points": trail_points}
