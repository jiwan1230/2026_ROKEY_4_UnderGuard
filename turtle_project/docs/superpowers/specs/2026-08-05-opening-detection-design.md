# Opening 감지 시 추가 조치 — camera_node 설계

작성일: 2026-08-05
패키지: `turtle_project` (ROS 2 Humble, ament_python)

## 목적

순찰 중 카메라 YOLO로 `opening`(침입구 후보)을 보면, Nav2로 그 앞 ~50cm까지
접근한 뒤 depth 편차로 **진짜 구멍(hole)인지** 판정한다. 판정 결과는 로그로 낸다.

Under-Guard 시스템의 Vision Part "opening 감지 시 추가 조치"에 해당하는 **기능만**
구현한다. 순찰 경로 주행, 맵 표시, 트랩 유도 등 상위 로직은 범위 밖.

> 참고: mini_turtle4는 별개 프로젝트다. 구현 **방식**만 참고하며, 필요한 지원
> 모듈은 이 워크스페이스에 새로 만든다 (import 의존 없음).

## 범위

- **포함**: opening 탐지 → Nav2 접근 → depth 편차 판정 → 로그
- **제외**: 순찰 주행, 맵/토픽 발행, spin 재탐색, 트랩 로직

## 입력

ROS 토픽 구독 (OAK-D 드라이버가 발행하는 것을 그대로 사용):

| 용도 | 토픽 | 타입 |
|------|------|------|
| RGB | `oakd/rgb/image_raw/compressed` | `CompressedImage` |
| Depth | `oakd/stereo/image_raw/compressedDepth` | `CompressedImage` (16UC1, mm) |
| CameraInfo | `oakd/stereo/camera_info` | `CameraInfo` |

RGB와 depth는 `ApproximateTimeSynchronizer`로 header.stamp를 짝지어 함께 받는다
(차가 움직일 때 프레임 시각이 어긋나 거리가 틀어지는 것을 방지).

## 파일 구성

새로 만드는 파일 3개:

### 1. `turtle_project/depth_math.py`
ROS 무관 순수 함수. 노드가 import.

- `decode_depth(data, fmt)` — compressedDepth → 16UC1 depth 배열(mm)
- `to_depth_px(u, v, rgb_shape, depth_shape)` — RGB 픽셀 → depth 픽셀
- `depth_at(depth_mm, u, v, patch)` — (u,v) 주변 patch 중앙값 → 미터 (무효 시 None)
- `deproject(u, v, z, K)` — depth 픽셀+거리 → 카메라 광학 프레임 3D
- **`depth_spread(depth_mm, x1, y1, x2, y2)`** (신규) — bbox 영역 유효 depth의
  퍼센타일 편차 `p90 − p10`(미터). 유효 픽셀 부족하면 None.

### 2. `turtle_project/nav_controller.py`
Nav2 주행 래퍼. 노드가 import.

- `approach_point(x, y, stop_dist)` — 물체(base_link 기준 x,y) 앞 stop_dist 지점
  `(gx, gy, yaw)`. 이미 안이면 None.
- `make_pose(frame, x, y, yaw)` — `PoseStamped` 조립
- `Navigator(node)` — `NavigateToPose` 액션 래퍼. `go(pose)`로 goal 전송,
  **도착(SUCCEEDED) 시 콜백**으로 상태머신에 알림. 한 번에 goal 하나.
  (mini_turtle4의 spin 재탐색 로직은 제외)

### 3. `turtle_project/camera_node.py`
상태머신 본체.

## 상태머신

```
SEARCHING ──opening 탐지+map좌표──> APPROACHING ──Nav2 SUCCEEDED──> VERIFYING
    ^                                                                   │
    └───────────────────── 판정 로그 후 복귀 ─────────────────────────┘
```

1. **SEARCHING** — YOLO로 `target_class` 탐지. 잡히면 bbox 중심 depth →
   `deproject` → TF(`base_link`/`map`)로 좌표 계산 후 APPROACHING.
2. **APPROACHING** — `approach_point`로 앞 `approach_dist` goal 발행.
   Nav2 result가 **SUCCEEDED**면 VERIFYING. 접근 중 새 탐지 무시.
3. **VERIFYING** — 도착 후 프레임에서 opening bbox 재검출 → `depth_spread` 계산.
   `depth_gap` 이상이면 `"진짜 opening 확인 (gap=…m)"`, 아니면 `"opening 아님"`.
   재검출 N프레임 실패 시 포기 로그 후 SEARCHING.

## 파라미터 (전부 기본값 있음)

| 이름 | 기본값 | 설명 |
|------|--------|------|
| `model_path` | `''` | YOLO/`.engine` 경로. 비었거나 로드 실패 시 탐지 스킵 |
| `target_class` | `'opening'` | 이 클래스만 목표 |
| `conf` | `0.6` | YOLO confidence |
| `approach_dist` | `0.5` | 물체 앞 몇 m에서 멈출지 (OAK-D 최소 depth 고려 튜닝 knob) |
| `depth_gap` | `0.05` | 이 이상 편차면 hole로 판정 (m) |
| `verify_timeout` | `30` (프레임) | VERIFYING에서 재검출 포기 기준 |

## 에러 처리

- **모델 없음/로드 실패**: 노드는 뜨고 경고 로그만, 탐지 스킵. `.engine` 나오면
  `model_path`만 주면 동작. 테스트는 `model_path:=best.pt target_class:=car`.
- **depth 무효** (너무 가깝거나 반사): 해당 탐지 스킵, 로그 남김.
- **TF 실패**: 경고 로그(throttle), 해당 프레임 스킵.
- **Nav2 서버 없음**: 경고 로그, goal 전송 실패 처리.

## 테스트

순수 함수만 `_self_check()`(assert 기반, `__main__`):

- `depth_math.py`: 기존 함수 + **`depth_spread`** (평평한 벽 → 작은 spread,
  구멍 패턴 → 큰 spread, 유효 픽셀 부족 → None)
- `nav_controller.py`: `approach_point`, `make_pose`

ROS/YOLO/하드웨어 경로는 self-check 불가 → 제외.

## ponytail 메모

- spin 재탐색 제외: 일회성 접근이라 불필요.
- import 재사용 대신 파일 신규 생성: 사용자 요청(별개 프로젝트, 자립형).
- 도착 판정은 Nav2 result(SUCCEEDED) — 거리 반경 판정보다 정확.
