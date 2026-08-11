import sqlite3
import tempfile
import time
import unittest
from pathlib import Path

from system_monitor.history_store import HistoryStore


class HistoryStoreTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        self.store = HistoryStore(root / "history.db", root / "captures")

    def test_empty_store_reports_zero_summary(self):
        self.assertEqual(self.store.summary(), {"detections": 0, "trail_points": 0})

    def test_record_and_list_detection_without_image(self):
        detection_id = self.store.record_detection(
            robot_id="robot4", object_type="ENTRY_POINT", map_x=1.5, map_y=-2.5,
            confidence=0.8, timestamp=1000.0, opening_id="O001",
            trap_id="T001", trap_installation_status="INSTALLED",
        )
        rows = self.store.list_detections()
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["id"], detection_id)
        self.assertEqual(rows[0]["robot_id"], "robot4")
        self.assertEqual(rows[0]["object_type"], "ENTRY_POINT")
        self.assertEqual((rows[0]["map_x"], rows[0]["map_y"]), (1.5, -2.5))
        self.assertEqual(rows[0]["opening_id"], "O001")
        self.assertEqual(rows[0]["trap_id"], "T001")
        self.assertEqual(rows[0]["trap_installation_status"], "INSTALLED")
        self.assertIsNone(rows[0]["image_url"])
        self.assertFalse(rows[0]["is_dummy"])

    def test_record_detection_with_image_saves_file_and_exposes_url(self):
        detection_id = self.store.record_detection(
            robot_id="robot6", object_type="LIVE_RODENT", map_x=0.0, map_y=0.0,
            image_bytes=b"\xff\xd8fake-jpeg", image_ext="jpg", is_dummy=True,
        )
        row = self.store.list_detections()[0]
        self.assertEqual(row["image_url"], f"/api/history/detections/{detection_id}/image")
        self.assertTrue(row["is_dummy"])

        path = self.store.image_path_for(detection_id)
        self.assertIsNotNone(path)
        self.assertEqual(path.read_bytes(), b"\xff\xd8fake-jpeg")

    def test_image_path_for_missing_or_imageless_detection_is_none(self):
        detection_id = self.store.record_detection(
            robot_id="robot4", object_type="DROPPINGS", map_x=0.0, map_y=0.0,
        )
        self.assertIsNone(self.store.image_path_for(detection_id))
        self.assertIsNone(self.store.image_path_for(99999))

    def test_list_detections_filters_by_time_type_and_robot(self):
        self.store.record_detection(
            robot_id="robot4", object_type="LIVE_RODENT", map_x=0, map_y=0, timestamp=100.0
        )
        self.store.record_detection(
            robot_id="robot6", object_type="ENTRY_POINT", map_x=0, map_y=0, timestamp=200.0
        )
        self.store.record_detection(
            robot_id="robot4", object_type="LIVE_RODENT", map_x=0, map_y=0, timestamp=300.0
        )

        self.assertEqual(len(self.store.list_detections(since=150.0)), 2)
        self.assertEqual(len(self.store.list_detections(until=150.0)), 1)
        self.assertEqual(len(self.store.list_detections(object_type="ENTRY_POINT")), 1)
        self.assertEqual(len(self.store.list_detections(robot_id="robot4")), 2)
        self.store.record_trail_point(
            robot_id="robot4", map_x=0, map_y=0, timestamp=100.0
        )
        self.store.record_trail_point(
            robot_id="robot6", map_x=0, map_y=0, timestamp=200.0
        )
        self.assertEqual(
            self.store.summary(since=150.0, robot_id="robot6"),
            {"detections": 1, "trail_points": 1},
        )
        self.assertEqual(
            self.store.summary(object_type="LIVE_RODENT"),
            {"detections": 2, "trail_points": 2},
        )

    def test_opening_trap_status_defaults_validates_and_filters(self):
        unknown_id = self.store.record_detection(
            robot_id="robot4", object_type="ENTRY_POINT", map_x=0, map_y=0
        )
        installed_id = self.store.record_detection(
            robot_id="robot6", object_type="ENTRY_POINT", map_x=1, map_y=1,
            opening_id="O002", trap_id="T002",
            trap_installation_status="installed",
        )
        rodent_id = self.store.record_detection(
            robot_id="robot4", object_type="LIVE_RODENT", map_x=2, map_y=2,
            opening_id="ignored", trap_id="ignored",
            trap_installation_status="INSTALLED",
        )

        rows = {row["id"]: row for row in self.store.list_detections()}
        self.assertEqual(
            rows[unknown_id]["trap_installation_status"], "UNKNOWN"
        )
        self.assertEqual(
            rows[installed_id]["trap_installation_status"], "INSTALLED"
        )
        self.assertIsNone(rows[rodent_id]["opening_id"])
        self.assertIsNone(rows[rodent_id]["trap_installation_status"])
        self.assertEqual(
            [row["id"] for row in self.store.list_detections(
                trap_installation_status="INSTALLED"
            )],
            [installed_id],
        )
        self.assertEqual(
            self.store.summary(trap_installation_status="INSTALLED")["detections"],
            1,
        )

        with self.assertRaises(ValueError):
            self.store.record_detection(
                robot_id="robot4", object_type="ENTRY_POINT", map_x=0, map_y=0,
                trap_installation_status="MOVED",
            )

    def test_existing_detection_schema_is_migrated_without_data_loss(self):
        root = Path(self.temp.name)
        db_path = root / "legacy.db"
        conn = sqlite3.connect(db_path)
        conn.execute(
            "CREATE TABLE detections ("
            "id INTEGER PRIMARY KEY AUTOINCREMENT, timestamp REAL NOT NULL, "
            "robot_id TEXT, object_type TEXT, map_x REAL, map_y REAL, "
            "confidence REAL, image_path TEXT, is_dummy INTEGER NOT NULL DEFAULT 0)"
        )
        conn.execute(
            "INSERT INTO detections "
            "(timestamp, robot_id, object_type, map_x, map_y, is_dummy) "
            "VALUES (1, 'robot4', 'ENTRY_POINT', 1, 2, 0)"
        )
        conn.commit()
        conn.close()

        migrated = HistoryStore(db_path, root / "legacy-captures")
        row = migrated.list_detections()[0]
        self.assertEqual(row["opening_id"], None)
        self.assertEqual(row["trap_installation_status"], "UNKNOWN")

    def test_list_detections_orders_newest_first(self):
        self.store.record_detection(
            robot_id="robot4", object_type="DROPPINGS", map_x=0, map_y=0, timestamp=100.0
        )
        self.store.record_detection(
            robot_id="robot4", object_type="DROPPINGS", map_x=0, map_y=0, timestamp=300.0
        )
        self.store.record_detection(
            robot_id="robot4", object_type="DROPPINGS", map_x=0, map_y=0, timestamp=200.0
        )
        timestamps = [row["timestamp"] for row in self.store.list_detections()]
        self.assertEqual(timestamps, [300.0, 200.0, 100.0])

    def test_get_trail_orders_oldest_first_and_filters_by_robot(self):
        self.store.record_trail_point(robot_id="robot4", map_x=0, map_y=0, timestamp=300.0)
        self.store.record_trail_point(robot_id="robot6", map_x=1, map_y=1, timestamp=100.0)
        self.store.record_trail_point(robot_id="robot4", map_x=2, map_y=2, timestamp=200.0)

        all_points = self.store.get_trail()
        self.assertEqual([p["timestamp"] for p in all_points], [100.0, 200.0, 300.0])

        robot4_only = self.store.get_trail(robot_id="robot4")
        self.assertEqual([p["map_x"] for p in robot4_only], [2, 0])

    def test_detection_timestamp_defaults_to_now_when_not_given(self):
        before = time.time()
        self.store.record_detection(
            robot_id="robot4", object_type="DROPPINGS", map_x=0, map_y=0
        )
        after = time.time()
        row = self.store.list_detections()[0]
        self.assertTrue(before <= row["timestamp"] <= after)

    def test_data_survives_reopening_the_same_db_file(self):
        root = Path(self.temp.name)
        store = HistoryStore(root / "persist.db", root / "persist-captures")
        store.record_detection(
            robot_id="robot4", object_type="LIVE_RODENT", map_x=1.0, map_y=1.0, timestamp=1.0
        )

        reopened = HistoryStore(root / "persist.db", root / "persist-captures")
        self.assertEqual(reopened.summary(), {"detections": 1, "trail_points": 0})


if __name__ == "__main__":
    unittest.main()
