import tempfile
import unittest
from pathlib import Path

from system_monitor.map_service import MapService


class MapServiceTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        root = Path(self.temp.name)
        self.pgm_path = root / "my_map.pgm"
        self.yaml_path = root / "my_map.yaml"
        self.pgm_path.write_bytes(b"P5\n2 2\n255\n" + bytes([0, 100, 205, 255]))
        self.yaml_path.write_text(
            "image: my_map.pgm\n"
            "resolution: 0.5\n"
            "origin: [-1.0, -2.0, 0.0]\n"
            "negate: 0\n",
            encoding="utf-8",
        )

    def tearDown(self):
        self.temp.cleanup()

    def test_loads_metadata_and_converts_pgm_to_png(self):
        service = MapService(self.yaml_path)

        metadata = service.describe("/api/map/image")

        self.assertTrue(metadata["available"])
        self.assertEqual((metadata["width"], metadata["height"]), (2, 2))
        self.assertEqual(metadata["resolution"], 0.5)
        self.assertEqual(metadata["origin"], [-1.0, -2.0, 0.0])
        self.assertTrue(service.png_bytes().startswith(b"\x89PNG\r\n\x1a\n"))

    def test_world_coordinates_are_converted_with_y_axis_flip(self):
        service = MapService(self.yaml_path)

        self.assertEqual(service.world_to_pixel(-1.0, -2.0), (0.0, 2.0))
        self.assertEqual(service.world_to_pixel(-0.5, -1.5), (1.0, 1.0))

    def test_missing_map_returns_fallback_description(self):
        service = MapService(Path(self.temp.name) / "missing.yaml")

        metadata = service.describe("/api/map/image")

        self.assertFalse(metadata["available"])
        self.assertIsNone(metadata["image_url"])


if __name__ == "__main__":
    unittest.main()
