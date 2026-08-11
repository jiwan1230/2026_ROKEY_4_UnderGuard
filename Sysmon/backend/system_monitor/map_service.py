"""ROS map YAML/PGM을 브라우저용 메타데이터와 PNG로 변환한다."""

from __future__ import annotations

import math
from io import BytesIO
from pathlib import Path
from typing import Any

import yaml
from PIL import Image, ImageOps


class MapService:
    """ROS 맵을 불러오고, 실제 좌표 (x, y)와 이미지 픽셀 좌표를 연결해주는 역할"""

    def __init__(self, yaml_path: str | Path, *, frame_id: str = "map") -> None:
        self.yaml_path = Path(yaml_path)
        self.frame_id = frame_id.strip("/") or "map"
        # 맵 파일이 변경되었는지 확인하기 위한 캐시 기준값
        self._cache_key: tuple[int, int] | None = None
        # 맵의 부가 정보
        self._metadata: dict[str, Any] | None = None
        self._png: bytes | None = None

    # 중요 #
    # 메타데이터와 이미지 URL 반환
    def describe(self, image_url: str) -> dict[str, Any]:
        '''맵을 먼저 로드하고, 
           실패하면 오류 정보를 반환하며, 
           성공하면 이미지 URL과 맵 메타데이터를 함께 반환'''
        
        try:
            self._load()
        except (OSError, ValueError) as exc:
            return {
                "available": False,
                "image_url": None,
                "error": str(exc),
            }

        # 맵 메타데이터는 resolution, origin, width, height 같은 정보
        assert self._metadata is not None
        return {
            "available": True,
            "image_url": image_url,
            **self._metadata,
        }

    # 중요 #
    # PNG 바이트 반환
    def png_bytes(self) -> bytes:
        """ROS의 PGM 맵 이미지를 (브라우저에서 보여줄 수 있는) PNG 데이터로 반환"""

        self._load()
        assert self._png is not None
        return self._png

    # 중요 #
    # 맵 실제 영역 반환
    def bounds(self) -> tuple[float, float, float, float]:
        '''맵의 시작 x,y 좌표와 실제 가로·세로 길이를 반환'''

        self._load()
        assert self._metadata is not None
        origin_x, origin_y, _ = self._metadata["origin"]
        bounds = self._metadata["bounds"]
        return origin_x, origin_y, bounds["local_width_m"], bounds["local_height_m"]

    # 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 # 
    def world_to_pixel(self, x: float, y: float) -> tuple[float, float]:
        """ROS의 실제 좌표 (x, y)를 웹 맵 이미지의 픽셀 좌표 (pixel_x, pixel_y)로 바꾼다.

        화면에 표시되는 이미지는 90도 반시계방향으로 회전되어 있으므로
        (ROTATE_90), 그 회전을 반영해 좌표를 변환한다.
        """

        self._load() #####
        assert self._metadata is not None
        # 맵의 시작 위치와 회전값을 가져옴
        origin_x, origin_y, origin_yaw = self._metadata["origin"]

        # 중요 #
        # 1) 맵 시작점에서 로봇까지 x,y방향으로 얼마나 떨어져 있는지 구함
        dx = float(x) - origin_x
        dy = float(y) - origin_y

        # 중요 #
        # 2) 회전되어 있는 맵에서 x방향 위치를 다시 계산
        # cos와 sin을 이용해서 맵의 회전각 origin_yaw를 보정하고, 
        # 로봇의 상대 좌표 dx, dy를 이미지 기준의 local_x 좌표로 변환
        local_x = math.cos(origin_yaw) * dx + math.sin(origin_yaw) * dy
        local_y = -math.sin(origin_yaw) * dx + math.cos(origin_yaw) * dy

        # 중요 #
        # 3) m 단위를 픽셀로 바꾼다. 원래는 "pixel_x=local_x, pixel_y=height-local_y"
        #    (y축만 뒤집음)였는데, _load()에서 PNG 자체를 90도 반시계방향으로
        #    돌려서 내려주므로(ROTATE_90) 좌표 변환도 그 회전을 반영해야 한다.
        #    x/y가 자리를 바꾸는 건 CW 회전과 같지만, CCW는 CW의 거울상이라
        #    두 축 모두 원점 반대편(가로/세로 끝)에서부터 잰다 — 그래서 CW
        #    버전과 달리 width/height를 빼는 보정이 남는다. (row,col) 인덱스로
        #    직접 검증된 공식: new_col=old_row, new_row=(W-1)-old_col.
        pixel_x = self._metadata["width"] - local_y / self._metadata["resolution"]
        pixel_y = self._metadata["height"] - local_x / self._metadata["resolution"]
        return pixel_x, pixel_y

    # 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 #
    def _load(self) -> None:
        '''ROS 맵의 YAML 파일과 이미지 파일을 읽고, 
           맵 정보를 정리한 뒤 PNG 형태로 변환해서, 
           메타데이터와 PNG 데이터를 캐시에 저장'''

        # 저장된 YAML 경로에 실제 파일이 있는지 확인
        if not self.yaml_path.is_file():
            raise ValueError(f"맵 YAML 파일을 찾을 수 없습니다: {self.yaml_path}")

        # 중요 1#      
        config = yaml.safe_load(self.yaml_path.read_text(encoding="utf-8")) #####
        if not isinstance(config, dict):
            raise ValueError("맵 YAML 내용이 올바르지 않습니다.")
        missing = {"image", "resolution", "origin"} - config.keys()
        if missing:
            # 빠진 항목 이름들을 문자열로 합침
            fields = ", ".join(sorted(missing))
            raise ValueError(f"맵 YAML 필수 항목이 없습니다: {fields}")

        image_path = Path(config["image"]) #####
        if not image_path.is_absolute():
            # YAML 파일이 있는 폴더를 기준으로 실제 이미지 경로를 만듬
            image_path = self.yaml_path.parent / image_path
        # 계산된 위치에 실제 맵 이미지가 있는지 확인
        if not image_path.is_file():
            raise ValueError(f"맵 이미지 파일을 찾을 수 없습니다: {image_path}")

        # YAML과 이미지가 변경되었는지 이전에 저장한 수정 시간과 현재 수정 시간이 같은지 비교
        # 불필요한 맵 재로딩을 방지하는 캐싱 처리
        cache_key = (
            self.yaml_path.stat().st_mtime_ns,
            image_path.stat().st_mtime_ns,
        )
        if self._cache_key == cache_key:
            return

        # 중요 2#
        # 맵 이미지 파일을 연다
        with Image.open(image_path) as source:
            # 이미지를 흑백 그레이스케일로 변환(통일 시킴)
            image = source.convert("L") #####

        # 중요 3#
        # YAML의 negate 설정을 확인합니다.
        if bool(config.get("negate", 0)):
            # 필요하면 이미지의 흑백을 반전
            image = ImageOps.invert(image)
        # 원본 ROS 맵은 보통 세로로 길게 나온다. 화면에는 가로(landscape)로
        # 보는 게 더 보기 편해서 90도 반시계방향으로 돌려서 내려준다 — 이
        # 회전은 world_to_pixel()의 좌표 변환, dashboard.js의
        # createMapProjection과 항상 함께 맞춰야 한다. (실제 room_map.pgm으로
        # numpy.rot90(arr, k=1)과 픽셀 단위로 비교 검증된 방향이다.)
        image = image.transpose(Image.Transpose.ROTATE_90)
        width, height = image.size
        resolution = float(config["resolution"]) #####
        origin = [float(value) for value in config["origin"]] #####
        
        # resolution이 정상적인 값인지 확인
        if resolution <= 0:
            raise ValueError("맵 resolution은 0보다 커야 합니다.")
        # origin 값이 정확히 3개인지 확인
        if len(origin) != 3:
            raise ValueError("맵 origin은 [x, y, yaw] 세 값이어야 합니다.")

        # 중요 4#
        # ROS 맵 정보를 웹 관제에서 사용할 수 있도록 정리한 메타데이터
        self._metadata = {
            # YAML 파일 이름에서 확장자를 제외한 이름
            "name": self.yaml_path.stem,
            # 
            "frame_id": self.frame_id,
            "width": width,
            "height": height,
            "resolution": resolution,
            # 맵의 실제 시작 좌표와 회전값
            "origin": origin,
            # 맵의 실제 물리적인 크기
            "bounds": {
                "local_width_m": width * resolution,
                "local_height_m": height * resolution,
            },
        }

        # 중요 5#
        # 메모리 안에 가상의 파일 공간을 만듦
        # PNG 이미지를 실제 파일로 저장하지 않고 메모리(RAM) 안에 임시 파일처럼 저장
        # 맵은 자주 변경되는 데이터가 아니기 때문에,
        # PNG를 메모리에 저장해두고 재사용해서, 브라우저 전달을 간단하고 빠르게 처리
        output = BytesIO()
        # output 안의 PNG 데이터를 bytes 형태로 꺼내서 _png에 저장
        image.save(output, format="PNG")
        self._png = output.getvalue()

        # 현재 YAML과 이미지의 수정 시간을 저장
        # 다음 _load() 호출 때 파일이 변경됐는지 비교하기 위해 사용
        self._cache_key = cache_key

'''
YAML 파일 확인
    ↓
YAML 읽기
    ↓
image / resolution / origin 확인
    ↓
PGM 이미지 읽기
    ↓
흑백 변환 및 필요 시 반전
    ↓
이미지 90도 회전
    ↓
width / height / resolution / origin 계산
    ↓
_metadata 생성
    ↓
PNG bytes 생성
    ↓
캐시에 저장
'''