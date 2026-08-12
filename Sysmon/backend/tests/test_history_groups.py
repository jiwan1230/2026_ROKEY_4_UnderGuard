"""시간대 묶음 로직 검증 — dashboard.js의 함수를 그대로 꺼내 node로 돌린다.

지도에 기록을 전부 겹쳐 그리면 어지러워서 "비슷한 시간대"끼리 나눠 보여주는데,
그 경계를 정하는 게 groupDetectionsByGap 하나다. JS 테스트 도구를 새로 들이는
대신 실제 소스에서 함수만 떼어 실행한다 — 사본을 만들지 않아 원본이 바뀌면
이 검증도 같이 바뀐다.
"""

import json
import shutil
import subprocess
import unittest
from pathlib import Path

_DASHBOARD_JS = (
    Path(__file__).resolve().parents[2] / "frontend" / "static" / "js" / "dashboard.js"
)


def _extract_function(source: str, name: str) -> str:
    """`function name(...) { ... }` 한 덩어리를 중괄호 균형으로 잘라낸다."""

    start = source.index(f"function {name}(")
    depth = 0
    for index in range(source.index("{", start), len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start : index + 1]
    raise AssertionError(f"{name}의 끝을 찾지 못했습니다.")


def _group(detections: list[dict], gap_sec: int = 300) -> list[dict]:
    """실제 JS 함수에 탐지 목록을 넣고 묶음 결과를 돌려받는다."""

    function = _extract_function(_DASHBOARD_JS.read_text(encoding="utf-8"),
                                 "groupDetectionsByGap")
    script = (
        f"{function}\n"
        f"const groups = groupDetectionsByGap({json.dumps(detections)}, {gap_sec});\n"
        "console.log(JSON.stringify(groups.map(g => "
        "({start: g.start, end: g.end, ids: g.items.map(d => d.id)}))));"
    )
    done = subprocess.run(
        ["node", "-e", script], capture_output=True, text=True, check=True
    )
    return json.loads(done.stdout)


@unittest.skipIf(shutil.which("node") is None, "node가 없어 JS 로직을 못 돌립니다.")
class HistoryGroupTest(unittest.TestCase):
    """탐지는 최신순으로 들어온다(DB가 detected_at DESC로 준다)."""

    @staticmethod
    def _dets(*timestamps):
        return [
            {"id": index + 1, "timestamp": ts} for index, ts in enumerate(timestamps)
        ]

    def test_close_detections_stay_in_one_group(self):
        groups = _group(self._dets(1000, 940, 880))    # 60초 간격

        self.assertEqual(len(groups), 1)
        self.assertEqual(groups[0]["ids"], [1, 2, 3])
        self.assertEqual((groups[0]["start"], groups[0]["end"]), (880, 1000))

    def test_a_long_quiet_gap_starts_a_new_group(self):
        groups = _group(self._dets(2000, 1940, 1000, 940))   # 가운데 940초 공백

        self.assertEqual([group["ids"] for group in groups], [[1, 2], [3, 4]])

    def test_gap_exactly_at_the_threshold_is_still_the_same_group(self):
        """경계값이 어느 쪽인지 애매하면 묶음 수가 상황마다 달라진다."""

        self.assertEqual(len(_group(self._dets(1000, 700))), 1)   # 정확히 300초
        self.assertEqual(len(_group(self._dets(1000, 699))), 2)   # 301초

    def test_detections_without_a_time_are_left_out(self):
        """시각을 모르면 어느 시간대인지 정할 수 없다 — 묶음에 넣지 않는다."""

        detections = [{"id": 1, "timestamp": 1000}, {"id": 2, "timestamp": None}]

        groups = _group(detections)

        self.assertEqual([group["ids"] for group in groups], [[1]])

    def test_no_detections_makes_no_groups(self):
        self.assertEqual(_group([]), [])


def _trail_for_group(group_index: int, groups: list[dict], trail: list[dict]) -> list[int]:
    """선택한 묶음에 걸리는 경로 점만 남기는 실제 JS 함수를 돌린다."""

    source = _DASHBOARD_JS.read_text(encoding="utf-8")
    script = (
        "const HISTORY_GROUP_TRAIL_PAD_SEC = 150;\n"
        f"const historyGroups = {json.dumps(groups)};\n"
        f"const historyTrail = {json.dumps(trail)};\n"
        f"const selectedHistoryGroupIndex = {group_index};\n"
        f"{_extract_function(source, 'visibleHistoryTrail')}\n"
        "console.log(JSON.stringify(visibleHistoryTrail().map(p => p.id)));"
    )
    done = subprocess.run(
        ["node", "-e", script], capture_output=True, text=True, check=True
    )
    return json.loads(done.stdout)


@unittest.skipIf(shutil.which("node") is None, "node가 없어 JS 로직을 못 돌립니다.")
class HistoryGroupTrailTest(unittest.TestCase):
    """묶음을 골랐으면 그 시간대의 경로만 그려야 지도가 깨끗해진다."""

    GROUPS = [{"start": 1000, "end": 1200}]      # 앞뒤 150초까지 함께 본다
    TRAIL = [
        {"id": 1, "timestamp": 700},             # 너무 이르다
        {"id": 2, "timestamp": 900},            # 시작 150초 전 — 접근 경로
        {"id": 3, "timestamp": 1100},           # 묶음 한가운데
        {"id": 4, "timestamp": 1350},           # 종료 150초 후 — 이탈 경로
        {"id": 5, "timestamp": 1600},           # 너무 늦다
    ]

    def test_only_the_selected_time_window_is_drawn(self):
        self.assertEqual(_trail_for_group(0, self.GROUPS, self.TRAIL), [2, 3, 4])

    def test_selecting_all_keeps_every_point(self):
        self.assertEqual(
            _trail_for_group(-1, self.GROUPS, self.TRAIL), [1, 2, 3, 4, 5]
        )


if __name__ == "__main__":
    unittest.main()
