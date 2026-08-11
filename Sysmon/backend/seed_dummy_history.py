#!/usr/bin/env python3
"""기록 조회 탭을 시연할 수 있도록 더미 탐지·이동경로 기록을 채워 넣는다.

실제 로봇의 쥐 감지→사진 촬영 배선은 아직 없다(detector_node.py의
_detect_rat이 TODO). 그게 끝날 때까지, 이 스크립트가 만든 가짜 기록으로
history_store.py + 기록 조회 UI를 먼저 개발·시연할 수 있게 한다.

몇 번을 실행해도 안전하다 — 매번 새 더미 데이터를 追加만 하며(is_dummy=1로
표시), 지우거나 실제 기록을 건드리지 않는다. 지우고 싶으면 DB 파일 자체를
삭제하면 된다(HISTORY_DB_PATH, 기본 data/history.db).

사용: cd Sysmon/backend && python3 seed_dummy_history.py
"""
from __future__ import annotations

import math
import random
import time
from io import BytesIO
from pathlib import Path

from PIL import Image, ImageDraw

from system_monitor.config import load_settings
from system_monitor.history_store import HistoryStore
from system_monitor.map_service import MapService
from system_monitor.risk_signals import DROPPINGS, ENTRY_POINT, LIVE_RODENT

# 실제 카메라가 없으니 색만 다른 사각형 이미지를 "증거 사진" 자리표시자로 쓴다.
_PLACEHOLDER_COLORS = {
    LIVE_RODENT: (200, 60, 60),
    ENTRY_POINT: (140, 90, 220),
    DROPPINGS: (150, 110, 60),
}


def _make_placeholder_image(object_type: str, when: float) -> bytes:
    color = _PLACEHOLDER_COLORS.get(object_type, (90, 90, 90))
    img = Image.new("RGB", (320, 240), color)
    draw = ImageDraw.Draw(img)
    stamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(when))
    draw.rectangle([8, 8, 312, 232], outline=(255, 255, 255), width=2)
    draw.text((20, 100), "DUMMY CAPTURE", fill=(255, 255, 255))
    draw.text((20, 120), object_type, fill=(255, 255, 255))
    draw.text((20, 140), stamp, fill=(255, 255, 255))
    buf = BytesIO()
    img.save(buf, format="JPEG")
    return buf.getvalue()


def _resolve(path: Path) -> Path:
    return path if path.is_absolute() else Path.cwd() / path


def seed(detection_count: int = 12, trail_points_per_robot: int = 60) -> None:
    settings = load_settings()
    store = HistoryStore(
        _resolve(settings.history_db_path), _resolve(settings.history_image_dir)
    )

    map_service = MapService(
        _resolve(settings.map_yaml_path), frame_id=settings.ros_interface.map_frame
    )
    try:
        origin_x, origin_y, width_m, height_m = map_service.bounds()
    except (OSError, ValueError):
        origin_x, origin_y, width_m, height_m = 0.0, 0.0, 6.0, 5.0
    margin = min(width_m, height_m) * 0.12
    x_lo, x_hi = origin_x + margin, origin_x + width_m - margin
    y_lo, y_hi = origin_y + margin, origin_y + height_m - margin

    robot_ids = [robot.robot_id for robot in settings.robots] or ["robot4", "robot6"]
    now = time.time()
    window_sec = 2 * 3600  # 지난 2시간을 시연 데이터로 채운다

    # 1) 탐지 기록 — LIVE_RODENT는 자리표시자 사진을 붙이고, 나머지는 사진 없이.
    object_types = [LIVE_RODENT, ENTRY_POINT, DROPPINGS]
    opening_index = 0
    for i in range(detection_count):
        ts = now - random.uniform(0, window_sec)
        object_type = random.choices(object_types, weights=[3, 2, 2])[0]
        x = round(random.uniform(x_lo, x_hi), 2)
        y = round(random.uniform(y_lo, y_hi), 2)
        image_bytes = (
            _make_placeholder_image(object_type, ts)
            if object_type == LIVE_RODENT
            else None
        )
        opening_id = None
        trap_id = None
        trap_installation_status = None
        if object_type == ENTRY_POINT:
            opening_index += 1
            opening_id = f"O{opening_index:03d}"
            trap_installation_status = random.choice(
                ["INSTALLED", "NOT_INSTALLED", "UNKNOWN"]
            )
            if trap_installation_status == "INSTALLED":
                trap_id = f"T{opening_index:03d}"
        store.record_detection(
            robot_id=random.choice(robot_ids),
            object_type=object_type,
            map_x=x,
            map_y=y,
            confidence=round(random.uniform(0.75, 0.97), 2),
            opening_id=opening_id,
            trap_id=trap_id,
            trap_installation_status=trap_installation_status,
            timestamp=ts,
            image_bytes=image_bytes,
            image_ext="jpg",
            is_dummy=True,
        )

    # 2) 이동 경로 — 로봇마다 방 안을 대충 순찰하는 것처럼 보이는 점 몇십 개.
    for robot_id in robot_ids:
        cx, cy = (x_lo + x_hi) / 2, (y_lo + y_hi) / 2
        radius = min(x_hi - x_lo, y_hi - y_lo) / 2 * random.uniform(0.5, 0.85)
        phase0 = random.uniform(0, 6.28)
        for i in range(trail_points_per_robot):
            t = i / trail_points_per_robot
            ts = now - window_sec + t * window_sec
            angle = phase0 + t * 6.28 * 3  # 창 시간 동안 세 바퀴 정도 순찰
            x = round(cx + radius * 0.9 * math.cos(angle), 2)
            y = round(cy + radius * 0.6 * math.sin(angle), 2)
            store.record_trail_point(
                robot_id=robot_id, map_x=x, map_y=y, timestamp=ts, is_dummy=True
            )

    print("더미 기록 생성 완료:", store.summary())
    print("DB:", store.db_path)
    print("이미지 폴더:", store.image_dir)


if __name__ == "__main__":
    seed()
