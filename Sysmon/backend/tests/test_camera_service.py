import unittest

from system_monitor.camera_service import CameraFrameStore


class CameraFrameStoreTest(unittest.TestCase):
    def setUp(self):
        self.store = CameraFrameStore()

    def test_unknown_robot_has_no_frame(self):
        self.assertIsNone(self.store.get("robot4"))
        self.assertIsNone(self.store.image_url_for("robot4"))

    def test_update_then_get_returns_latest_frame(self):
        self.store.update("robot4", b"jpeg-bytes-1", "jpeg")
        self.store.update("robot4", b"jpeg-bytes-2", "jpeg")

        frame = self.store.get("robot4")

        self.assertEqual(frame.content, b"jpeg-bytes-2")
        self.assertEqual(frame.format, "jpeg")
        self.assertGreater(frame.received_at, 0)

    def test_image_url_for_only_reports_robots_with_a_cached_frame(self):
        self.store.update("robot4", b"jpeg-bytes", "jpeg")

        self.assertEqual(self.store.image_url_for("robot4"), "/api/camera/robot4/frame")
        self.assertIsNone(self.store.image_url_for("robot6"))


if __name__ == "__main__":
    unittest.main()
