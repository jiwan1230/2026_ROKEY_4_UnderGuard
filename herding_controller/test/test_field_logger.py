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
    target_positions = [np.array([5.9, 5.0])]  # 0.9m away, inside a 1.0m panic distance
    target_velocities = [np.array([-0.4, 0.0])]  # moving toward the robot: violates rule 2
    count = detect_rule_violations(robot_positions, target_positions, target_velocities, panic_distance_m=1.0, dt=0.2)
    assert count == 1


# --- Regression tests added during self-review ---


def test_select_capture_zone_never_prints_to_stdout_or_stderr(capsys):
    """Blinding requirement: the operator must never see the capture zone on the console."""
    rng = np.random.default_rng(1)
    for _ in range(20):
        select_capture_zone(rng)
    captured = capsys.readouterr()
    assert captured.out == ""
    assert captured.err == ""


def test_field_logger_log_trial_never_prints_capture_zone(capsys):
    """log_trial writes capture_zone_id only to the CSV file, never to the console."""
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
    """A field laptop crash/restart mid-session should not require the operator to pre-create dirs."""
    csv_path = tmp_path / "nested" / "does" / "not" / "exist" / "field_log.csv"
    logger = FieldLogger(str(csv_path))
    logger.log_trial(
        trial_id=1, condition="CONTROL-A", capture_zone_id=0, start_time=0.0, end_time=1.0,
        success=True, min_robot_target_dist=0.5, rule_violation_count=0, note="",
    )
    assert csv_path.exists()


def test_field_logger_appends_without_duplicating_header_across_process_restarts(tmp_path):
    """Simulates the field laptop crashing and the logging process being restarted mid-session."""
    csv_path = tmp_path / "field_log.csv"

    logger1 = FieldLogger(str(csv_path))
    logger1.log_trial(
        trial_id=1, condition="TREATMENT", capture_zone_id=1, start_time=0.0, end_time=5.0,
        success=True, min_robot_target_dist=0.3, rule_violation_count=0, note="",
    )
    del logger1  # process "crashes" here

    # A fresh process re-opens the logger pointed at the same existing CSV.
    logger2 = FieldLogger(str(csv_path))
    logger2.log_trial(
        trial_id=2, condition="TREATMENT", capture_zone_id=1, start_time=5.0, end_time=9.0,
        success=False, min_robot_target_dist=0.05, rule_violation_count=1, note="restarted",
    )

    with open(csv_path, newline="") as f:
        raw_lines = f.readlines()
    header_lines = [line for line in raw_lines if line.startswith("trial_id,")]
    assert len(header_lines) == 1  # header written exactly once, not duplicated

    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 2
    assert rows[0]["trial_id"] == "1"
    assert rows[1]["trial_id"] == "2"
    assert rows[1]["note"] == "restarted"


def test_field_logger_success_written_as_unambiguous_int_not_truthy_string(tmp_path):
    """success=False must not round-trip as a truthy value for naive analysis scripts.

    A naive analysis script often does `if row["success"]: ...`. If success were written
    via str(bool) it becomes the *string* "False", and bool("False") is True in Python -
    silently corrupting every failure row. Writing 0/1 avoids that specific footgun.
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
    """A timestep with zero visible/tracked robots must be skipped, not crash on argmin([])."""
    robot_positions = [[], [np.array([5.0, 5.0])]]
    target_positions = [np.array([5.9, 5.0]), np.array([5.9, 5.0])]
    target_velocities = [np.array([-0.4, 0.0]), np.array([-0.4, 0.0])]
    count = detect_rule_violations(robot_positions, target_positions, target_velocities, panic_distance_m=1.0, dt=0.2)
    assert count == 1  # only the second timestep (with a robot present) counts


def test_detect_rule_violations_mismatched_list_lengths_raises_instead_of_silently_truncating():
    """Parallel lists of different lengths indicate a caller bug; silent zip()-truncation would
    silently undercount safety-rule violations in a human-subjects field report."""
    robot_positions = [[np.array([5.0, 5.0])], [np.array([5.0, 5.0])]]
    target_positions = [np.array([5.9, 5.0])]
    target_velocities = [np.array([-0.4, 0.0])]
    with pytest.raises(ValueError):
        detect_rule_violations(robot_positions, target_positions, target_velocities, panic_distance_m=1.0, dt=0.2)


def test_detect_rule_violations_closest_robot_is_the_one_evaluated_not_farther_threatened_one():
    """Documents the spec's defined semantics: with two robots simultaneously within panic
    distance, only the *closest* robot's direction is evaluated for rule 2, even if the target
    is moving toward the farther (but still close) robot instead. This matches the interface
    spec literally ("target's velocity points toward the closest robot") - it is not a bug,
    but is worth pinning down with a test since it means a real safety-relevant approach toward
    a second robot can go uncounted."""
    robot_a = np.array([5.0, 5.0])  # 0.3 m away: closest
    robot_b = np.array([5.95, 5.0])  # 0.95 m away: also within panic distance, but farther
    robot_positions = [[robot_a, robot_b]]
    target_positions = [np.array([5.3, 5.0])]
    # Moving toward robot_b (+x direction), i.e. away from the closest robot_a.
    target_velocities = [np.array([0.5, 0.0])]
    count = detect_rule_violations(robot_positions, target_positions, target_velocities, panic_distance_m=1.0, dt=0.2)
    assert count == 0  # per spec: only the closest robot (robot_a) is evaluated, and target moves away from it
