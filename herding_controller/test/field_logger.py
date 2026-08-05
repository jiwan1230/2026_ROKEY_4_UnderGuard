"""미니카 현장 프로토콜을 위한 블라인드 포획 구역 선택 및 시행별 CSV 로깅."""
import csv
import os

import numpy as np

CAPTURE_ZONE_CANDIDATES: list[tuple[float, float]] = [
    (3.0, 3.0), (7.0, 3.0), (3.0, 7.0), (7.0, 7.0),
]

_FIELDNAMES = [
    "trial_id", "condition", "capture_zone_id", "start_time", "end_time", "success",
    "duration_sec", "min_robot_target_dist", "rule_violation_count", "note",
]


def select_capture_zone(rng: np.random.Generator) -> tuple[float, float]:
    """4개의 고정 포획 구역 후보 중 하나를 선택한다. 호출자는 이 값을 출력해서는 안 된다(조작자 블라인딩)."""
    index = int(rng.integers(0, len(CAPTURE_ZONE_CANDIDATES)))
    return CAPTURE_ZONE_CANDIDATES[index]


class FieldLogger:
    """현장 시행마다 CSV 행을 하나씩 추가한다. 포획 구역 값을 콘솔에 절대 출력하지 않는다."""

    def __init__(self, csv_path: str) -> None:
        self.csv_path = csv_path
        parent_dir = os.path.dirname(csv_path)
        if parent_dir:
            os.makedirs(parent_dir, exist_ok=True)
        if not os.path.exists(csv_path):
            with open(csv_path, "w", newline="") as f:
                csv.DictWriter(f, fieldnames=_FIELDNAMES).writeheader()

    def log_trial(
        self, trial_id: int, condition: str, capture_zone_id: int, start_time: float, end_time: float,
        success: bool, min_robot_target_dist: float, rule_violation_count: int, note: str = "",
    ) -> None:
        """한 시행의 결과를 추가한다. capture_zone_id는 이 파일에만 기록되고 절대 출력되지 않는다."""
        with open(self.csv_path, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=_FIELDNAMES)
            writer.writerow({
                "trial_id": trial_id, "condition": condition, "capture_zone_id": capture_zone_id,
                "start_time": start_time, "end_time": end_time, "success": int(bool(success)),
                "duration_sec": end_time - start_time, "min_robot_target_dist": min_robot_target_dist,
                "rule_violation_count": rule_violation_count, "note": note,
            })


def detect_rule_violations(
    robot_positions: list, target_positions: list, target_velocities: list,
    reaction_distance_m: float, dt: float,
) -> int:
    """로봇이 reaction_distance_m 이내에 있는데도 타겟이 그쪽으로 이동하는 주기를 센다 (규칙 2).

    `reaction_distance_m`은 조작자 프로토콜의 규칙 2 반경, 즉 설정값의
    `flee_reaction_distance_m`(1.0 m, "로봇이 약 1m 안으로 들어오면 도망친다")을 의미하며,
    `panic_distance_m`(0.35 m, 플래너 자체의 후퇴 임계값)이 아니다. 후자를 넘기면
    위반을 과소 집계하여 20% 위반 시행 제외 분석을 왜곡하게 된다.
    """
    if not (len(robot_positions) == len(target_positions) == len(target_velocities)):
        raise ValueError(
            "robot_positions, target_positions, and target_velocities must have the same "
            f"number of time steps, got {len(robot_positions)}, {len(target_positions)}, "
            f"{len(target_velocities)}"
        )
    violations = 0
    for robots_at_t, target_pos, target_vel in zip(robot_positions, target_positions, target_velocities):
        if not robots_at_t:
            continue
        distances = [np.linalg.norm(target_pos - r) for r in robots_at_t]
        closest_index = int(np.argmin(distances))
        closest_dist = distances[closest_index]
        if closest_dist >= reaction_distance_m:
            continue
        away = target_pos - robots_at_t[closest_index]
        speed = np.linalg.norm(target_vel)
        if speed < 1e-6:
            continue
        toward_robot = np.dot(target_vel / speed, away) < 0
        if toward_robot:
            violations += 1
    return violations
