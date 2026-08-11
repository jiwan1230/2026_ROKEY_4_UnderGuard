import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from system_monitor.replay_manager import ReplayManager
from system_monitor.rosbag_converter import (
    ConversionError,
    SampleEvent,
    TopicConfig,
    build_replay_document,
    build_trial,
    events_from_message,
)
from system_monitor.state_manager import StateManager


def _pose_message(x, y, frame="map", *, odometry=False):
    position = SimpleNamespace(x=x, y=y)
    pose = SimpleNamespace(position=position)
    if odometry:
        pose = SimpleNamespace(pose=pose)
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame),
        pose=pose,
    )


class RosbagConverterTest(unittest.TestCase):
    def test_build_trial_samples_irregular_events_by_recorded_time(self):
        events = [
            SampleEvent(10.0, "driver", [0.0, 0.0]),
            SampleEvent(10.0, "blocker", [2.0, 2.0]),
            SampleEvent(10.0, "target", [1.0, 1.0]),
            SampleEvent(10.0, "state", "SEARCH"),
            SampleEvent(10.05, "driver", [0.1, 0.0]),
            SampleEvent(10.10, "target", [1.1, 1.0]),
            SampleEvent(10.20, "state", "HERDING"),
            SampleEvent(10.20, "capture_progress", 0.5),
            SampleEvent(10.20, "driver_goal", [0.8, 0.9]),
            SampleEvent(10.25, "state", "CAPTURED"),
        ]

        trial = build_trial(
            events,
            sample_period=0.1,
            model="field_algorithm_v1",
            goal_name="top",
            seed=42,
        )

        self.assertEqual([frame["t"] for frame in trial["frames"]], [0.0, 0.1, 0.2, 0.25])
        self.assertEqual(trial["frames"][1]["driver"], [0.1, 0.0])
        self.assertEqual(trial["frames"][2]["state"], "HERD")
        self.assertEqual(trial["frames"][2]["capture_progress"], 0.5)
        self.assertEqual(trial["frames"][-1]["state"], "CAPTURED")
        self.assertEqual(trial["frames"][-1]["capture_progress"], 1.0)
        self.assertTrue(trial["success"])
        self.assertEqual(trial["discovery_time"], 0.2)
        self.assertEqual(trial["model"], "field_algorithm_v1")
        # Blocker 목표 토픽이 없으면 해당 시점의 실제 위치를 안전한 대체값으로 쓴다.
        self.assertEqual(trial["frames"][0]["blocker_goal"], [2.0, 2.0])

    def test_build_trial_reports_missing_required_position(self):
        with self.assertRaisesRegex(ConversionError, "target"):
            build_trial(
                [
                    SampleEvent(1.0, "driver", [0.0, 0.0]),
                    SampleEvent(1.0, "blocker", [1.0, 1.0]),
                ]
            )

    def test_replay_document_can_copy_layout_and_append_existing_trials(self):
        old_trial = {"model": "old", "frames": []}
        new_trial = {"model": "new", "frames": [{"t": 0.0}]}
        base = {
            "map_image": "data:image/png;base64,example",
            "photo_frame": {"x_low": -1, "x_high": 1, "y_low": -2, "y_high": 2},
            "traps": {"top": [0.0, 1.0]},
            "capture_radius": 0.35,
            "trials": [old_trial],
        }

        document = build_replay_document(
            new_trial, base_document=base, append_existing_trials=True
        )

        self.assertEqual(document["map_image"], base["map_image"])
        self.assertEqual(document["traps"], base["traps"])
        self.assertEqual([trial["model"] for trial in document["trials"]], ["old", "new"])
        document["traps"]["top"][0] = 99.0
        self.assertEqual(base["traps"]["top"][0], 0.0)

    def test_message_adapter_reads_current_main_topics(self):
        topics = TopicConfig()

        driver = events_from_message(
            "/robot4/odom", _pose_message(1.2, 3.4, odometry=True), 1.0, topics
        )
        target = events_from_message(
            "/fleet/event", SimpleNamespace(data="rat_detected:2.50:-1.25"), 1.1, topics
        )
        state = events_from_message(
            "/fleet/status", SimpleNamespace(data="robot4:HERDING:80"), 1.2, topics
        )
        other_robot = events_from_message(
            "/fleet/status", SimpleNamespace(data="robot6:PATROLLING:70"), 1.3, topics
        )

        self.assertEqual(driver[0].value, [1.2, 3.4])
        self.assertEqual(driver[0].frame_id, "map")
        self.assertEqual(target[0].value, [2.5, -1.25])
        self.assertEqual(state[0].value, "HERD")
        self.assertEqual(other_robot, [])

    def test_captured_event_sets_position_state_progress_and_success(self):
        events = events_from_message(
            "/fleet/event", SimpleNamespace(data="rat_captured:1.0:2.0"), 5.0, TopicConfig()
        )

        self.assertEqual(
            [event.field for event in events],
            ["target", "state", "capture_progress", "success"],
        )
        self.assertTrue(events[-1].value)

    def test_generated_document_is_accepted_by_replay_manager(self):
        trial = build_trial(
            [
                SampleEvent(1.0, "driver", [0.0, 0.0]),
                SampleEvent(1.0, "blocker", [2.0, 2.0]),
                SampleEvent(1.0, "target", [1.0, 1.0]),
                SampleEvent(1.1, "state", "CAPTURED"),
            ]
        )
        document = build_replay_document(trial)

        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "converted.json"
            path.write_text(json.dumps(document), encoding="utf-8")
            state = StateManager(
                [("robot4", "SCOUT"), ("robot6", "SCOUT")],
                offline_timeout_sec=100,
            )
            manager = ReplayManager(
                state, ["robot4", "robot6"], frames_path=path
            )

            self.assertTrue(manager.available)
            self.assertEqual(manager.history_record()["trial"]["model"], "rosbag_recording")
            self.assertTrue(manager.history_record()["trial"]["success"])


if __name__ == "__main__":
    unittest.main()
