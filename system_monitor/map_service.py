"""ROS map YAML/PGM을 브라우저용 메타데이터와 PNG로 변환한다."""

from __future__ import annotations

import math
from io import BytesIO
from pathlib import Path
from typing import Any

import yaml
from PIL import Image, ImageOps


class MapService:
    """정적 ROS 맵을 로드하고 실제 좌표와 이미지 좌표의 관계를 제공한다."""

    def __init__(self, yaml_path: str | Path, *, frame_id: str = "map") -> None:
        self.yaml_path = Path(yaml_path)
        self.frame_id = frame_id.strip("/") or "map"
        self._cache_key: tuple[int, int] | None = None
        self._metadata: dict[str, Any] | None = None
        self._png: bytes | None = None

    def describe(self, image_url: str) -> dict[str, Any]:
        """대시보드가 맵을 그릴 때 필요한 메타데이터를 반환한다.

        입력: PNG를 제공하는 Flask API URL이다.
        출력: 사용 가능 여부, 크기, 해상도, 원점, 실제 좌표 범위다.
        사용: ``GET /api/map`` 응답을 만들 때 호출한다.
        """

        try:
            self._load()
        except (OSError, ValueError) as exc:
            return {
                "available": False,
                "image_url": None,
                "error": str(exc),
            }

        assert self._metadata is not None
        return {
            "available": True,
            "image_url": image_url,
            **self._metadata,
        }

    def png_bytes(self) -> bytes:
        """PGM을 변환한 브라우저 표시용 PNG 바이트를 반환한다."""

        self._load()
        assert self._png is not None
        return self._png

    def world_to_pixel(self, x: float, y: float) -> tuple[float, float]:
        """map 좌표(m)를 원본 PGM 픽셀 좌표로 변환한다.

        입력: map frame 기준 ``x``, ``y`` 좌표다.
        출력: 이미지 좌측 상단 기준 픽셀 ``x``, ``y``다.
        사용: 프런트엔드 Canvas도 동일한 계산식으로 로봇과 마커를 배치한다.
        """

        self._load()
        assert self._metadata is not None
        origin_x, origin_y, origin_yaw = self._metadata["origin"]
        # 1) map 좌표를 origin만큼 평행이동한다.
        dx = float(x) - origin_x
        dy = float(y) - origin_y
        # 2) origin_yaw만큼 반대로 회전시켜 이미지 축과 나란히 맞춘다
        #    (표준 2D 회전 변환의 역행렬).
        local_x = math.cos(origin_yaw) * dx + math.sin(origin_yaw) * dy
        local_y = -math.sin(origin_yaw) * dx + math.cos(origin_yaw) * dy
        # 3) m 단위를 픽셀로 바꾸고, y축은 이미지가 위→아래로 증가하므로
        #    ROS 기준(아래→위 증가)과 반대로 뒤집는다.
        pixel_x = local_x / self._metadata["resolution"]
        pixel_y = self._metadata["height"] - local_y / self._metadata["resolution"]
        return pixel_x, pixel_y

    def _load(self) -> None:
        if not self.yaml_path.is_file():
            raise ValueError(f"맵 YAML 파일을 찾을 수 없습니다: {self.yaml_path}")

        config = yaml.safe_load(self.yaml_path.read_text(encoding="utf-8"))
        if not isinstance(config, dict):
            raise ValueError("맵 YAML 내용이 올바르지 않습니다.")
        missing = {"image", "resolution", "origin"} - config.keys()
        if missing:
            fields = ", ".join(sorted(missing))
            raise ValueError(f"맵 YAML 필수 항목이 없습니다: {fields}")

        image_path = Path(config["image"])
        if not image_path.is_absolute():
            image_path = self.yaml_path.parent / image_path
        if not image_path.is_file():
            raise ValueError(f"맵 이미지 파일을 찾을 수 없습니다: {image_path}")

        cache_key = (
            self.yaml_path.stat().st_mtime_ns,
            image_path.stat().st_mtime_ns,
        )
        if self._cache_key == cache_key:
            return

        with Image.open(image_path) as source:
            image = source.convert("L")
        if bool(config.get("negate", 0)):
            image = ImageOps.invert(image)
        width, height = image.size
        resolution = float(config["resolution"])
        origin = [float(value) for value in config["origin"]]
        if resolution <= 0:
            raise ValueError("맵 resolution은 0보다 커야 합니다.")
        if len(origin) != 3:
            raise ValueError("맵 origin은 [x, y, yaw] 세 값이어야 합니다.")

        self._metadata = {
            "name": self.yaml_path.stem,
            "frame_id": self.frame_id,
            "width": width,
            "height": height,
            "resolution": resolution,
            "origin": origin,
            "bounds": {
                "local_width_m": width * resolution,
                "local_height_m": height * resolution,
            },
        }
        output = BytesIO()
        image.save(output, format="PNG")
        self._png = output.getvalue()
        self._cache_key = cache_key
