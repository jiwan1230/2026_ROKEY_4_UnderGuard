import csv

import numpy as np
import pytest

from test.field_logger import (
    CAPTURE_ZONE_CANDIDATES,
    FieldLogger,
    detect_rule_violations,
    select_capture_zone,
)


def test_select_capture_zone_returns_one_of_the_four_candidates():
    rng = np.random.default_rng(0)
    zone = select_capture_zone(rng)
    assert zone in CAPTURE_ZONE_CANDIDATES


def test_field_logger_writes_expected_columns(tmp_path):
    csv_path = tmp_path / "field_log.csv"
    logger = FieldLogger(str(csv_path))
    logger.log_trial(
        trial_id=1, condition="TREATMENT", capture_zone_id=2, start_time=0.0, end_time=12.5,
        success=True, min_robot_target_dist=0.4, rule_violation_count=0, note="",
    )
    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))
    assert rows[0]["trial_id"] == "1"
    assert rows[0]["duration_sec"] == "12.5"
    assert set(rows[0].keys()) == {
        "trial_id", "condition", "capture_zone_id", "start_time", "end_time",
        "success", "duration_sec", "min_robot_target_dist", "rule_violation_count", "note",
    }


def test_detect_rule_violations_counts_target_moving_toward_close_robot():
    robot_positions = [[np.array([5.0, 5.0])]]
    target_positions = [np.array([5.9, 5.0])]  # 0.9m 거리, 1.0m 패닉 거리 이내
    target_velocities = [np.array([-0.4, 0.0])]  # 로봇 쪽으로 이동: 규칙 2 위반
    count = detect_rule_violations(robot_positions, target_positions, target_velocities, reaction_distance_m=1.0, dt=0.2)
    assert count == 1


# --- 자체 검토 중 추가된 회귀 테스트 ---


def test_select_capture_zone_never_prints_to_stdout_or_stderr(capsys):
    """블라인딩 요구사항: 운영자는 콘솔에서 캡처 구역을 절대 볼 수 없어야 한다."""
    rng = np.random.default_rng(1)
    for _ in range(20):
        select_capture_zone(rng)
    captured = capsys.readouterr()
    assert captured.out == ""
    assert captured.err == ""


def test_field_logger_log_trial_never_prints_capture_zone(capsys):
    """log_trial는 capture_zone_id를 CSV 파일에만 기록하며, 콘솔에는 절대 출력하지 않는다."""
    import tempfile
    import os as _os

    with tempfile.TemporaryDirectory() as d:
        csv_path = _os.path.join(d, "log.csv")
        logger = FieldLogger(csv_path)
        logger.log_trial(
            trial_id=1, condition="TREATMENT", capture_zone_id=3, start_time=0.0, end_time=1.0,
            success=False, min_robot_target_dist=0.1, rule_violation_count=0, note="",
        )
        captured = capsys.readouterr()
        assert captured.out == ""
        assert captured.err == ""


def test_field_logger_creates_parent_directory_if_missing(tmp_path):
    """세션 도중 현장 노트북이 충돌/재시작되더라도 운영자가 미리 디렉터리를 만들 필요가 없어야 한다."""
    csv_path = tmp_path / "nested" / "does" / "not" / "exist" / "field_log.csv"
    logger = FieldLogger(str(csv_path))
    logger.log_trial(
        trial_id=1, condition="CONTROL-A", capture_zone_id=0, start_time=0.0, end_time=1.0,
        success=True, min_robot_target_dist=0.5, rule_violation_count=0, note="",
    )
    assert csv_path.exists()


def test_field_logger_appends_without_duplicating_header_across_process_restarts(tmp_path):
    """세션 도중 현장 노트북이 충돌하고 로깅 프로세스가 재시작되는 상황을 시뮬레이션한다."""
    csv_path = tmp_path / "field_log.csv"

    logger1 = FieldLogger(str(csv_path))
    logger1.log_trial(
        trial_id=1, condition="TREATMENT", capture_zone_id=1, start_time=0.0, end_time=5.0,
        success=True, min_robot_target_dist=0.3, rule_violation_count=0, note="",
    )
    del logger1  # 여기서 프로세스가 "충돌"함

    # 새 프로세스가 동일한 기존 CSV를 가리키는 로거를 다시 연다.
    logger2 = FieldLogger(str(csv_path))
    logger2.log_trial(
        trial_id=2, condition="TREATMENT", capture_zone_id=1, start_time=5.0, end_time=9.0,
        success=False, min_robot_target_dist=0.05, rule_violation_count=1, note="restarted",
    )

    with open(csv_path, newline="") as f:
        raw_lines = f.readlines()
    header_lines = [line for line in raw_lines if line.startswith("trial_id,")]
    assert len(header_lines) == 1  # 헤더가 정확히 한 번만 기록되며 중복되지 않음

    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 2
    assert rows[0]["trial_id"] == "1"
    assert rows[1]["trial_id"] == "2"
    assert rows[1]["note"] == "restarted"


def test_field_logger_success_written_as_unambiguous_int_not_truthy_string(tmp_path):
    """success=False는 단순한 분석 스크립트에서 참 값으로 왕복 변환되어서는 안 된다.

    단순한 분석 스크립트는 흔히 `if row["success"]: ...`와 같이 작성한다. success를
    str(bool)로 기록하면 *문자열* "False"가 되는데, Python에서 bool("False")는 True이므로
    모든 실패 행이 조용히 손상된다. 0/1로 기록하면 이 특정 함정을 피할 수 있다.
    """
    csv_path = tmp_path / "field_log.csv"
    logger = FieldLogger(str(csv_path))
    logger.log_trial(
        trial_id=1, condition="CONTROL-B", capture_zone_id=0, start_time=0.0, end_time=1.0,
        success=False, min_robot_target_dist=0.2, rule_violation_count=0, note="",
    )
    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))
    assert rows[0]["success"] == "0"
    assert int(rows[0]["success"]) == 0


def test_detect_rule_violations_empty_robot_list_at_one_timestep_does_not_crash():
    """보이는/추적 중인 로봇이 0개인 타임스텝은 건너뛰어야 하며, argmin([])에서 크래시가 나서는 안 된다."""
    robot_positions = [[], [np.array([5.0, 5.0])]]
    target_positions = [np.array([5.9, 5.0]), np.array([5.9, 5.0])]
    target_velocities = [np.array([-0.4, 0.0]), np.array([-0.4, 0.0])]
    count = detect_rule_violations(robot_positions, target_positions, target_velocities, reaction_distance_m=1.0, dt=0.2)
    assert count == 1  # 로봇이 존재하는 두 번째 타임스텝만 카운트됨


def test_detect_rule_violations_mismatched_list_lengths_raises_instead_of_silently_truncating():
    """길이가 다른 병렬 리스트는 호출자 측의 버그를 나타낸다; zip()에 의한 조용한 절단은
    피험자 대상 현장 보고서에서 안전 규칙 위반을 조용히 과소 집계하게 된다."""
    robot_positions = [[np.array([5.0, 5.0])], [np.array([5.0, 5.0])]]
    target_positions = [np.array([5.9, 5.0])]
    target_velocities = [np.array([-0.4, 0.0])]
    with pytest.raises(ValueError):
        detect_rule_violations(robot_positions, target_positions, target_velocities, reaction_distance_m=1.0, dt=0.2)


def test_detect_rule_violations_closest_robot_is_the_one_evaluated_not_farther_threatened_one():
    """스펙에 정의된 의미를 문서화한다: 두 로봇이 동시에 패닉 거리 이내에 있을 때,
    타겟이 (여전히 가깝지만) 더 먼 로봇 쪽으로 이동하고 있더라도 규칙 2 평가에는
    *가장 가까운* 로봇의 방향만 사용된다. 이는 인터페이스 스펙의 문구
    ("타겟의 속도가 가장 가까운 로봇을 향한다")를 그대로 따른 것으로 버그가 아니지만,
    두 번째 로봇을 향한 실제 안전 관련 접근이 카운트되지 않을 수 있다는 의미이므로
    테스트로 명확히 고정해 둘 가치가 있다."""
    robot_a = np.array([5.0, 5.0])  # 0.3 m 거리: 가장 가까움
    robot_b = np.array([5.95, 5.0])  # 0.95 m 거리: 패닉 거리 이내지만 더 멂
    robot_positions = [[robot_a, robot_b]]
    target_positions = [np.array([5.3, 5.0])]
    # robot_b 쪽(+x 방향)으로 이동, 즉 가장 가까운 robot_a로부터는 멀어짐.
    target_velocities = [np.array([0.5, 0.0])]
    count = detect_rule_violations(robot_positions, target_positions, target_velocities, reaction_distance_m=1.0, dt=0.2)
    assert count == 0  # 스펙에 따라: 가장 가까운 로봇(robot_a)만 평가되며, 타겟은 그로부터 멀어짐
