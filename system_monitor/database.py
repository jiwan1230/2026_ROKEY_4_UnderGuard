"""SQLite 연결, 초기 스키마, 탐지/사건 영속화를 제공한다."""

from __future__ import annotations

import sqlite3
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Iterator

from .risk_signals import normalize_risk_signal
from .security import hash_password

VALID_REVIEW_STATUSES = {"UNREVIEWED", "REVIEWED", "ACTIONED", "FALSE_POSITIVE"}


class Database:
    """요청마다 짧은 SQLite 연결을 열어 스레드 간 연결 공유를 피한다."""

    def __init__(self, path: str | Path):
        self.path = str(path)

    @contextmanager
    def connect(self) -> Iterator[sqlite3.Connection]:
        """커밋/롤백/닫기를 보장하는 SQLite 연결 컨텍스트를 반환한다."""

        conn = sqlite3.connect(self.path, timeout=10)
        conn.row_factory = sqlite3.Row
        conn.execute("PRAGMA foreign_keys = ON")
        try:
            yield conn
            conn.commit()
        except Exception:
            conn.rollback()
            raise
        finally:
            conn.close()

    def initialize(self) -> None:
        """테이블·인덱스와 최초 관리자 계정을 멱등하게 생성한다."""

        with self.connect() as conn:
            conn.executescript(
                """
                CREATE TABLE IF NOT EXISTS users (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    username TEXT NOT NULL UNIQUE,
                    password_hash TEXT NOT NULL,
                    role TEXT NOT NULL DEFAULT 'OPERATOR',
                    created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
                );

                CREATE TABLE IF NOT EXISTS detections (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    robot_id TEXT NOT NULL,
                    object_type TEXT NOT NULL,
                    confidence REAL,
                    distance REAL,
                    map_x REAL,
                    map_y REAL,
                    source TEXT,
                    image_path TEXT,
                    animal_type TEXT,
                    review_status TEXT NOT NULL DEFAULT 'UNREVIEWED',
                    memo TEXT,
                    detected_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
                );

                CREATE TABLE IF NOT EXISTS system_events (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    robot_id TEXT,
                    severity TEXT NOT NULL,
                    event_type TEXT NOT NULL,
                    message TEXT NOT NULL,
                    created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
                );

                CREATE TABLE IF NOT EXISTS traps (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    robot_id TEXT NOT NULL,
                    map_x REAL NOT NULL,
                    map_y REAL NOT NULL,
                    status TEXT NOT NULL DEFAULT 'INSTALLED',
                    installed_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
                );

                CREATE INDEX IF NOT EXISTS idx_detection_type
                    ON detections(object_type);
                CREATE INDEX IF NOT EXISTS idx_detection_robot
                    ON detections(robot_id);
                CREATE INDEX IF NOT EXISTS idx_detection_time
                    ON detections(detected_at);
                CREATE INDEX IF NOT EXISTS idx_trap_time
                    ON traps(installed_at);
                """
            )
            # 이전 Mock/ROS 라벨도 이후 조회에서 같은 위험신호로 취급한다.
            for old_label in ("rc_car", "rat", "mouse"):
                conn.execute(
                    "UPDATE detections SET object_type = ? WHERE LOWER(object_type) = ?",
                    ("LIVE_RODENT", old_label),
                )
            for old_label in ("rat_hole", "hole"):
                conn.execute(
                    "UPDATE detections SET object_type = ? WHERE LOWER(object_type) = ?",
                    ("ENTRY_POINT", old_label),
                )
            conn.execute(
                "UPDATE detections SET object_type = ? "
                "WHERE LOWER(object_type) IN ('droppings', 'dropping')",
                ("DROPPINGS",),
            )
            exists = conn.execute(
                "SELECT 1 FROM users WHERE username = ?", ("admin",)
            ).fetchone()
            if not exists:
                conn.execute(
                    "INSERT INTO users(username, password_hash, role) VALUES (?, ?, ?)",
                    ("admin", hash_password("admin123"), "ADMIN"),
                )

    def find_user(self, username: str) -> dict[str, Any] | None:
        """로그인 정보를 조회하며, 사용자가 없으면 ``None``을 반환한다."""

        with self.connect() as conn:
            row = conn.execute(
                "SELECT id, username, password_hash, role FROM users WHERE username = ?",
                (username,),
            ).fetchone()
            return dict(row) if row else None

    def insert_detection(self, data: dict[str, Any]) -> int:
        """탐지 한 건을 저장한다.

        입력: ``robot_id``와 ``object_type``이 필수인 탐지 딕셔너리다.
        출력: SQLite가 발급한 탐지 레코드 ID다.
        사용: Mock/ROS 구분 없이 ``process_detection``에서 호출한다.
        """

        data = dict(data)
        data["object_type"] = normalize_risk_signal(data.get("object_type"))
        if not data.get("review_status"):
            data["review_status"] = "UNREVIEWED"
        fields = (
            "robot_id",
            "object_type",
            "confidence",
            "distance",
            "map_x",
            "map_y",
            "source",
            "image_path",
            "animal_type",
            "review_status",
            "memo",
        )
        values = [data.get(field) for field in fields]
        if not data.get("robot_id") or not data.get("object_type"):
            raise ValueError("robot_id와 object_type은 필수입니다.")
        with self.connect() as conn:
            cursor = conn.execute(
                f"INSERT INTO detections({', '.join(fields)}) "
                f"VALUES ({', '.join('?' for _ in fields)})",
                values,
            )
            return int(cursor.lastrowid)

    def search_detections(
        self,
        *,
        robot_id: str | None = None,
        object_type: str | None = None,
        review_status: str | None = None,
        detected_after: str | None = None,
        detected_before: str | None = None,
        limit: int = 200,
    ) -> list[dict[str, Any]]:
        """선택 필터에 맞는 탐지를 최신순으로 조회한다.

        입력: 로봇·객체·검토 상태, UTC 시간 범위와 최대 건수다.
        출력: JSON 직렬화 가능한 탐지 딕셔너리 목록이다.
        사용: ``GET /api/detections``의 검색 결과 생성에 사용한다.
        """

        clauses: list[str] = []
        params: list[Any] = []
        for field, value in (
            ("robot_id", robot_id),
            ("object_type", normalize_risk_signal(object_type) if object_type else None),
            ("review_status", review_status),
        ):
            if value:
                clauses.append(f"{field} = ?")
                params.append(value)
        if detected_after:
            clauses.append("detected_at >= ?")
            params.append(detected_after)
        if detected_before:
            clauses.append("detected_at < ?")
            params.append(detected_before)
        where = f"WHERE {' AND '.join(clauses)}" if clauses else ""
        params.append(max(1, min(limit, 1000)))
        with self.connect() as conn:
            rows = conn.execute(
                f"SELECT * FROM detections {where} "
                "ORDER BY detected_at DESC, id DESC LIMIT ?",
                params,
            ).fetchall()
            return [dict(row) for row in rows]

    def update_detection_status(
        self, detection_id: int, status: str, memo: str = ""
    ) -> bool:
        """탐지 검토 상태와 메모를 갱신하고 대상 존재 여부를 반환한다."""

        if status not in VALID_REVIEW_STATUSES:
            raise ValueError(f"지원하지 않는 review_status: {status}")
        with self.connect() as conn:
            cursor = conn.execute(
                "UPDATE detections SET review_status = ?, memo = ? WHERE id = ?",
                (status, memo, detection_id),
            )
            return cursor.rowcount == 1

    def insert_event(self, data: dict[str, Any]) -> int:
        """타임라인 사건을 영구 저장하고 발급된 레코드 ID를 반환한다."""

        with self.connect() as conn:
            cursor = conn.execute(
                """
                INSERT INTO system_events(robot_id, severity, event_type, message)
                VALUES (?, ?, ?, ?)
                """,
                (
                    data.get("robot_id"),
                    data.get("severity", "INFO"),
                    data.get("event_type", "SYSTEM"),
                    data["message"],
                ),
            )
            return int(cursor.lastrowid)

    def insert_trap(self, data: dict[str, Any]) -> int:
        """설치된 트랩의 로봇·지도 좌표를 저장하고 ID를 반환한다."""

        if not data.get("robot_id") or data.get("map_x") is None:
            raise ValueError("트랩 robot_id와 map_x/map_y는 필수입니다.")
        if data.get("map_y") is None:
            raise ValueError("트랩 robot_id와 map_x/map_y는 필수입니다.")
        with self.connect() as conn:
            cursor = conn.execute(
                """
                INSERT INTO traps(robot_id, map_x, map_y, status)
                VALUES (?, ?, ?, ?)
                """,
                (
                    data["robot_id"],
                    float(data["map_x"]),
                    float(data["map_y"]),
                    data.get("status", "INSTALLED"),
                ),
            )
            return int(cursor.lastrowid)

    def search_traps(self, limit: int = 100) -> list[dict[str, Any]]:
        """최근 설치 트랩을 최신순으로 조회한다."""

        with self.connect() as conn:
            rows = conn.execute(
                "SELECT * FROM traps ORDER BY installed_at DESC, id DESC LIMIT ?",
                (max(1, min(limit, 1000)),),
            ).fetchall()
            return [dict(row) for row in rows]

    def clear_operational_data(self) -> dict[str, int]:
        """사용자와 스키마는 유지하고 수집된 운영 데이터만 삭제한다.

        입력: 없음. 출력: 탐지·사건·트랩별 삭제 건수다.
        사용: 관리자용 Mock/ROS 데이터 초기화 경계에서 호출한다.
        세 테이블 삭제와 ID 시퀀스 초기화는 한 트랜잭션으로 처리된다.
        """

        tables = {
            "detections": "detections",
            "events": "system_events",
            "traps": "traps",
        }
        with self.connect() as conn:
            deleted = {
                label: int(conn.execute(f"SELECT COUNT(*) FROM {table}").fetchone()[0])
                for label, table in tables.items()
            }
            for table in tables.values():
                conn.execute(f"DELETE FROM {table}")
            conn.execute(
                "DELETE FROM sqlite_sequence WHERE name IN (?, ?, ?)",
                tuple(tables.values()),
            )
            return deleted
