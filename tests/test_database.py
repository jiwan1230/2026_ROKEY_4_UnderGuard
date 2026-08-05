import tempfile
import unittest
from pathlib import Path

from system_monitor.database import Database
from system_monitor.security import verify_password


class DatabaseTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.db = Database(Path(self.temp.name) / "test.db")
        self.db.initialize()

    def tearDown(self):
        self.temp.cleanup()

    def test_default_admin(self):
        user = self.db.find_user("admin")
        self.assertIsNotNone(user)
        self.assertTrue(verify_password("admin123", user["password_hash"]))

    def test_detection_crud(self):
        detection_id = self.db.insert_detection({
            "robot_id": "robot4",
            "object_type": "rat_hole",
            "confidence": 0.91,
            "map_x": 1.0,
            "map_y": 2.0,
            "review_status": "UNREVIEWED",
        })
        rows = self.db.search_detections(object_type="rat_hole")
        self.assertEqual(rows[0]["id"], detection_id)
        self.assertEqual(rows[0]["object_type"], "ENTRY_POINT")
        self.assertTrue(self.db.update_detection_status(detection_id, "REVIEWED"))
        self.assertEqual(self.db.search_detections()[0]["review_status"], "REVIEWED")

        self.assertTrue(self.db.update_detection_status(detection_id, "ACTIONED", "덫 설치 완료"))
        updated = self.db.search_detections()[0]
        self.assertEqual(updated["review_status"], "ACTIONED")
        self.assertEqual(updated["memo"], "덫 설치 완료")

    def test_rejects_invalid_review_status(self):
        with self.assertRaises(ValueError):
            self.db.update_detection_status(1, "INVALID")

    def test_detection_date_range(self):
        detection_id = self.db.insert_detection({
            "robot_id": "robot4",
            "object_type": "rc_car",
            "review_status": "UNREVIEWED",
        })
        with self.db.connect() as conn:
            conn.execute(
                "UPDATE detections SET detected_at = ? WHERE id = ?",
                ("2026-08-04 16:05:00", detection_id),
            )
        self.assertEqual(
            len(self.db.search_detections(
                detected_after="2026-08-04 15:00:00",
                detected_before="2026-08-04 17:00:00",
            )),
            1,
        )
        self.assertEqual(
            self.db.search_detections(detected_after="2026-08-05 00:00:00"),
            [],
        )

    def test_risk_signal_aliases_are_stored_with_canonical_names(self):
        aliases = {
            "rc_car": "LIVE_RODENT",
            "rat_hole": "ENTRY_POINT",
            "droppings": "DROPPINGS",
        }
        for alias, canonical in aliases.items():
            detection_id = self.db.insert_detection(
                {"robot_id": "robot4", "object_type": alias}
            )
            row = self.db.search_detections(object_type=canonical)[0]
            self.assertEqual(row["id"], detection_id)
            self.assertEqual(row["object_type"], canonical)

    def test_trap_location_is_persisted(self):
        trap_id = self.db.insert_trap(
            {"robot_id": "robot4", "map_x": 1.25, "map_y": 2.5}
        )

        trap = self.db.search_traps()[0]

        self.assertEqual(trap["id"], trap_id)
        self.assertEqual((trap["map_x"], trap["map_y"]), (1.25, 2.5))
        self.assertEqual(trap["status"], "INSTALLED")


if __name__ == "__main__":
    unittest.main()
