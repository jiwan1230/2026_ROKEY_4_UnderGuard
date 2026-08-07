"""depth 카메라 좌표/편차 계산. ROS 무관, 노드가 import해서 사용."""
import cv2
import numpy as np

PATCH = 5   # depth 샘플 반경 (픽셀). 한 점만 보면 0(무효)이 잘 나옴


def decode_depth(data, fmt):
    """compressedDepth CompressedImage.data -> 16UC1 depth 배열 (mm).

    compressedDepth는 PNG 앞에 12바이트 ConfigHeader가 붙어 있고 cv_bridge가
    이걸 못 벗긴다. 16UC1은 양자화 없이 raw PNG라 헤더만 잘라내면 그대로 mm
    값이 나온다. 헤더 없는 일반 compressed도 통과시킨다.
    """
    buf = np.frombuffer(data, np.uint8)
    if 'compressedDepth' in fmt:
        buf = buf[12:]
    return cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)


def to_depth_px(u, v, rgb_shape, depth_shape):
    """RGB 픽셀 -> depth 픽셀. 두 이미지 해상도가 달라서 필요.

    RGB와 depth는 화각이 같게 맞춰져 있어 해상도만 비례로 맞추면 픽셀이 대응한다.
    """
    rh, rw = rgb_shape[:2]
    dh, dw = depth_shape[:2]
    return int(u / rw * dw), int(v / rh * dh)


def depth_at(depth_mm, u, v, patch=PATCH):
    """(u, v) 주변 patch의 유효값 중앙값 -> 미터. 유효값 없으면 None."""
    h, w = depth_mm.shape
    win = depth_mm[max(0, v - patch):min(h, v + patch + 1),
                   max(0, u - patch):min(w, u + patch + 1)]
    win = win[win > 0]
    return float(np.median(win)) / 1000.0 if win.size else None


def deproject(u, v, z, K):
    """depth 픽셀 + 거리 -> 카메라 광학 프레임 3D (x우, y하, z전방)."""
    return ((u - K[0, 2]) * z / K[0, 0],
            (v - K[1, 2]) * z / K[1, 1],
            z)


def side_px(margin_m, z, fx):
    """실세계 좌우 마진(m) -> 거리 z(m)에서의 픽셀 수. z나 fx 없으면 0.

    bbox가 구멍에 딱 맞으면 안이 다 '먼' depth라 편차가 안 생긴다. 옆 벽을
    포함하려고 박스를 좌우로 넓힐 때 쓴다. 픽셀폭 = fx * margin / z.
    """
    if not z or not fx:
        return 0
    return int(round(fx * margin_m / z))


def depth_spread(depth_mm, x1, y1, x2, y2, min_valid=30, side=0):
    """bbox 영역 유효 depth의 p90 - p10 (미터). 유효 픽셀 부족하면 None.

    진짜 구멍이면 구멍 안쪽(멀다)과 테두리 벽(가깝다)이 섞여 편차가 크고,
    평평한 벽이면 depth가 고르니 편차가 작다. min-max 대신 퍼센타일이라
    depth 노이즈 몇 픽셀이 튀어도 안 흔들린다.

    side: 좌우로 넓힐 픽셀 (구멍에 bbox가 딱 맞을 때 옆 벽까지 포함). 상하는 그대로.
    """
    h, w = depth_mm.shape
    x1 = max(0, int(x1) - side)
    x2 = min(w, int(x2) + side)
    y1, y2 = max(0, int(y1)), min(h, int(y2))
    roi = depth_mm[y1:y2, x1:x2]
    valid = roi[roi > 0]
    if valid.size < min_valid:
        return None
    p90, p10 = np.percentile(valid, [90, 10])
    return float(p90 - p10) / 1000.0


def _self_check():
    d = np.zeros((400, 640), np.uint16)
    d[195:205, 315:325] = 1500                 # 중앙에 1.5m 블록
    assert depth_at(d, 320, 200) == 1.5
    assert depth_at(d, 10, 10) is None         # 전부 0 -> 무효
    assert depth_at(d, 0, 0) is None           # 경계에서 안 터짐

    # 구멍: 중심은 stereo 무효(0), 테두리 벽만 유효 -> 좁은 패치는 None,
    # 넓히면 벽 깊이가 잡힌다 (detector _box_to_map 폴백이 기대는 성질)
    holey = np.zeros((100, 100), np.uint16)
    holey[:, :30] = holey[:, 70:] = 1000        # 좌우 벽 1m, 가운데는 구멍
    assert depth_at(holey, 50, 50) is None      # 중심 5px 패치 -> 전부 0
    assert depth_at(holey, 50, 50, patch=50) == 1.0

    assert to_depth_px(125, 125, (250, 250), (400, 640)) == (320, 200)
    assert to_depth_px(0, 0, (250, 250), (400, 640)) == (0, 0)

    K = np.array([[500., 0., 320.], [0., 500., 200.], [0., 0., 1.]])
    assert deproject(320, 200, 2.0, K) == (0.0, 0.0, 2.0)   # 정중앙
    x, y, z = deproject(420, 200, 2.0, K)                   # 오른쪽 100px
    assert abs(x - 0.4) < 1e-9 and abs(y) < 1e-9 and z == 2.0

    # decode_depth: 12바이트 헤더 벗김 + 헤더 없는 경우
    dd = np.zeros((704, 704), np.uint16)
    dd[340:365, 340:365] = 1234
    png = cv2.imencode('.png', dd)[1].tobytes()
    out = decode_depth(b'\x00' * 12 + png, '16UC1; compressedDepth png')
    assert out.dtype == np.uint16 and depth_at(out, 352, 352) == 1.234
    assert depth_at(decode_depth(png, '16UC1; png'), 352, 352) == 1.234

    # depth_spread: 평평한 벽 -> 작은 편차
    wall = np.full((100, 100), 1000, np.uint16)
    assert depth_spread(wall, 0, 0, 100, 100) == 0.0
    # 구멍 패턴: 절반은 1m 벽, 절반은 1.5m 구멍 안쪽 -> p90-p10 = 0.5m
    hole = np.full((100, 100), 1000, np.uint16)
    hole[:, 50:] = 1500
    assert abs(depth_spread(hole, 0, 0, 100, 100) - 0.5) < 1e-9
    # 유효 픽셀 부족 -> None
    assert depth_spread(np.zeros((100, 100), np.uint16), 0, 0, 100, 100) is None
    # bbox가 이미지 밖으로 나가도 클램프되어 안 터짐
    assert depth_spread(wall, -10, -10, 200, 200) == 0.0

    # side_px: fx*margin/z. fx=500, 0.05m, z=1m -> 25px. z 없으면 0
    assert side_px(0.05, 1.0, 500) == 25
    assert side_px(0.05, 4.0, 500) == 6    # 멀수록 픽셀 적음 (500*0.05/4=6.25)
    assert side_px(0.05, None, 500) == 0
    assert side_px(0.05, 1.0, 0) == 0
    # side 마진: 구멍(먼 값)에 bbox가 딱 맞아 내부는 편차 0이지만, 좌우로 넓혀
    # 옆 벽(가까운 값)을 포함하면 편차가 생긴다
    scene = np.full((100, 100), 1000, np.uint16)   # 벽 1m
    scene[:, 40:60] = 1500                          # 가운데 구멍 1.5m
    assert depth_spread(scene, 40, 0, 60, 100) == 0.0          # bbox 딱 맞음 -> 편차 0
    assert abs(depth_spread(scene, 40, 0, 60, 100, side=10) - 0.5) < 1e-9  # 옆 벽 포함
    print('depth_math self-check ok')


if __name__ == '__main__':
    _self_check()
