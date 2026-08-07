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
        # 원본(회전 전) 3x2 픽셀 — 가로/세로가 안 같아야 회전 시 width/height가
        # 실제로 뒤바뀌는지 제대로 검증할 수 있다.
        self.pgm_path.write_bytes(
            b"P5\n3 2\n255\n" + bytes([0, 50, 100, 150, 200, 255])
        )
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
        # PNG는 화면 표시용으로 90도 시계방향 회전되어 나가므로, 원본
        # 3x2(width x height) 픽셀이 2x3으로 뒤바뀐다.
        self.assertEqual((metadata["width"], metadata["height"]), (2, 3))
        self.assertEqual(metadata["resolution"], 0.5)
        self.assertEqual(metadata["origin"], [-1.0, -2.0, 0.0])
        self.assertTrue(service.png_bytes().startswith(b"\x89PNG\r\n\x1a\n"))

    def test_world_coordinates_are_converted_with_90_degree_rotation(self):
        service = MapService(self.yaml_path)

        # origin과 정확히 겹치는 점 — 회전 전이었다면 (0, height)였겠지만,
        # 회전 후에는 회전된 이미지의 좌상단 (0, 0)이 된다.
        self.assertEqual(service.world_to_pixel(-1.0, -2.0), (0.0, 0.0))
        # local_x != local_y인 비대칭 케이스로 "회전 때문에 x/y가 실제로
        # 자리를 바꿨는지"를 확인한다(대칭 케이스는 우연히 통과할 수 있음).
        self.assertEqual(service.world_to_pixel(-1.0, -1.5), (1.0, 0.0))
        self.assertEqual(service.world_to_pixel(-0.5, -2.0), (0.0, 1.0))

    def test_missing_map_returns_fallback_description(self):
        service = MapService(Path(self.temp.name) / "missing.yaml")

        metadata = service.describe("/api/map/image")

        self.assertFalse(metadata["available"])
        self.assertIsNone(metadata["image_url"])


if __name__ == "__main__":
    unittest.main()
