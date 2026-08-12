# Blocker 반발항 지연 게이팅 + 페어드 어블레이션 스크립트 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `EscapeModel._robot_repulsion()`에서 Blocker(로봇 2)의 기여에 거리 기반
활성화 임계값을 추가하고, 능동/고정(frozen) Blocker를 동일 시드로 페어링해
rescue/regression을 세는 재사용 가능한 어블레이션 스크립트를 저장소에
커밋한다.

**Architecture:** `EscapeModelConfig`/`HerdingConfig`에 새 필드
`robot_repulsion_activation_distance_m`(기본값 `inf` = 게이팅 없음, 완전
하위호환)를 추가하고, `_robot_repulsion()` 내부에서 `robot_positions`의
인덱스 1(호출 규약상 항상 Blocker — `herding_core.py`가
`[robot1_pos, robot2_pos]` 순서로 넘김)에만 거리 게이팅을 적용한다. 이후
`HerdingCore._stabilize_blocking_point`를 몽키패치해 Blocker를 스폰 위치에
고정하는 "frozen" 조건을 만들고, `test/run_validation.py`의 기존
`make_real_map_config`/`run_trial_real_map`을 그대로 재사용해 능동/frozen을
동일 시드로 비교하는 스크립트를 작성한다.

**Tech Stack:** Python 3, numpy, scipy.stats(`binomtest`), pytest, 기존
`herding_controller`/`test` 패키지 구조 그대로 재사용.

## Global Constraints

- 신규 필드의 기본값은 반드시 기존 동작과 100% 동일해야 한다(하위호환) —
  `robot_repulsion_activation_distance_m: float = float("inf")`.
- `config/herding_params.yaml`은 이번 계획에서 변경하지 않는다 — 스윕 결과로
  채택할 값이 정해지기 전까지는 코드 레벨 기본값으로만 존재한다
  (스펙 문서 3-3 참고).
- 기존 유닛테스트 전부(142개+) 회귀 없이 통과해야 한다.
- 브랜치는 `algorithm/intelligence1-herding-blocker-redesign`을 그대로 사용,
  새 브랜치를 만들지 않는다.
- 작업 디렉터리 루트는 `/home/sunwook/2026_ROKEY_4_UnderGuard/intelligence1_algorithm/herding_controller/`
  (모든 `pytest`/`python3` 명령은 이 디렉터리에서 실행).

---

### Task 1: `EscapeModel._robot_repulsion()` 거리 게이팅

**Files:**
- Modify: `herding_controller/herding_controller/escape_model.py:16-25` (`EscapeModelConfig`), `escape_model.py:125-147` (`_robot_repulsion`)
- Test: `herding_controller/test/test_escape_model.py`

**Interfaces:**
- Consumes: 없음(기존 `EscapeModel`/`EscapeModelConfig` 그대로)
- Produces: `EscapeModelConfig.robot_repulsion_activation_distance_m: float`
  (기본값 `float("inf")`) — Task 2가 `HerdingConfig`에서 이 필드로 값을
  전달할 때 씀. `EscapeModel._robot_repulsion(target_pos, robot_positions)`
  시그니처는 변경 없음(내부 게이팅만 추가).

- [ ] **Step 1: 실패하는 테스트 작성**

`herding_controller/test/test_escape_model.py`의 `make_model` 헬퍼에
`robot_repulsion_activation_distance_m` 파라미터를 추가하고, 아래 3개
테스트를 파일 끝에 추가한다.

```python
def make_model(grid=None, robot_repulsion_activation_distance_m=float("inf")):
    grid = grid or GridMap(GridConfig(resolution_m=0.25, width_cells=40, height_cells=40))
    config = EscapeModelConfig(
        wall_follow_p=0.70, wall_hug_p=0.20, center_p=0.10,
        momentum_weight=0.4, robot_repulsion_weight=1.5,
        wall_detect_radius_cells=1, escape_route_top_k=3,
        robot_repulsion_activation_distance_m=robot_repulsion_activation_distance_m,
    )
    return EscapeModel(config, grid), grid
```

(위는 기존 `make_model` 함수를 이 내용으로 **교체**하는 것 — 기존 두 위치
호출부 `make_model()`, `make_model(grid)`는 새 파라미터에 기본값이 있으므로
그대로 동작한다.)

파일 끝에 추가:

```python
def test_robot_repulsion_gates_second_robot_beyond_activation_distance():
    """Blocker(robot_positions의 두 번째 원소)가 활성화 거리보다 멀면 기여가 0이어야 한다."""
    model, _ = make_model(robot_repulsion_activation_distance_m=1.0)
    target_pos = np.array([5.0, 5.0])
    driver_pos = np.array([4.5, 5.0])   # 0.5m -- 인덱스 0(Driver)은 게이팅 안 받음
    blocker_far = np.array([5.0, 8.0])  # 3.0m > 1.0m 활성화 거리

    with_far_blocker = model._robot_repulsion(target_pos, [driver_pos, blocker_far])
    driver_only = model._robot_repulsion(target_pos, [driver_pos])
    assert np.allclose(with_far_blocker, driver_only)


def test_robot_repulsion_includes_second_robot_within_activation_distance():
    """Blocker가 활성화 거리 안이면 게이팅 없을 때와 동일하게 기여해야 한다."""
    gated_model, _ = make_model(robot_repulsion_activation_distance_m=1.0)
    ungated_model, _ = make_model(robot_repulsion_activation_distance_m=float("inf"))
    target_pos = np.array([5.0, 5.0])
    driver_pos = np.array([4.5, 5.0])
    blocker_near = np.array([5.0, 5.5])  # 0.5m < 1.0m 활성화 거리

    gated = gated_model._robot_repulsion(target_pos, [driver_pos, blocker_near])
    ungated = ungated_model._robot_repulsion(target_pos, [driver_pos, blocker_near])
    assert np.allclose(gated, ungated)


def test_robot_repulsion_default_activation_distance_does_not_gate():
    """기본값(inf)에서는 아주 먼 두 번째 로봇도 여전히(약하게) 기여해야 한다 -- 하위호환 확인."""
    model, _ = make_model()  # 기본값 = inf
    target_pos = np.array([5.0, 5.0])
    driver_pos = np.array([4.5, 5.0])
    blocker_far = np.array([5.0, 20.0])  # 15m

    with_blocker = model._robot_repulsion(target_pos, [driver_pos, blocker_far])
    driver_only = model._robot_repulsion(target_pos, [driver_pos])
    assert not np.allclose(with_blocker, driver_only)
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd herding_controller && python3 -m pytest test/test_escape_model.py -v`
Expected: 새로 추가한 3개 테스트가 `TypeError: EscapeModelConfig.__init__() got an
unexpected keyword argument 'robot_repulsion_activation_distance_m'`로 FAIL
(나머지 기존 테스트는 여전히 PASS).

- [ ] **Step 3: 최소 구현 작성**

`escape_model.py`의 `EscapeModelConfig`를 아래로 교체(필드 하나 추가):

```python
@dataclass
class EscapeModelConfig:
    """벽 추종(thigmotaxis) 기본 가중치와 로봇 반발/관성 항."""
    wall_follow_p: float
    wall_hug_p: float
    center_p: float
    momentum_weight: float
    robot_repulsion_weight: float
    wall_detect_radius_cells: int
    escape_route_top_k: int
    # Blocker(robot_positions의 두 번째 원소)가 이 거리(m)보다 멀면
    # _robot_repulsion() 계산에서 그 로봇의 기여를 아예 제외한다. 기본값
    # inf는 게이팅 없음(기존 동작과 완전히 동일) -- 트러블슈팅 노트
    # 11-9/11-10/12 참고: Blocker의 존재가 몰이 초반부터 표적 경로에
    # 간접 영향을 줘, 한참 뒤 로봇 A 혼자 막다른 길에 갇히는 나비효과를
    # 만든다는 가설을 검증하기 위한 필드.
    robot_repulsion_activation_distance_m: float = float("inf")
```

`_robot_repulsion()`을 아래로 교체(로봇 인덱스 게이팅 추가, docstring에
한 문단 추가):

```python
    def _robot_repulsion(self, target_pos: np.ndarray, robot_positions: list[np.ndarray]) -> np.ndarray:
        """로봇이 가까울수록, 그 로봇과 반대 방향의 도주 확률을 더해준다.

        각 로봇에 대해 `away`(로봇→타겟 단위벡터)와의 내적이 클수록(그
        방향이 "로봇으로부터 멀어지는" 방향에 가까울수록) 가산 가중치를
        더 준다. `np.clip(..., 0.0, None)`으로 음수 내적(로봇 쪽으로
        향하는 방향)은 0으로 잘라내 감점이 아니라 단순 무가산 처리한다 --
        여러 로봇의 위협이 겹칠 때 서로 상쇄되어 "덜 위험해 보이는" 왜곡을
        막기 위함이다. `weight = robot_repulsion_weight / dist`로 거리에
        반비례시키는 것은 "가까운 로봇일수록 훨씬 급하게 도망친다"는
        직관 -- 위협이 2배 가까워지면 그 방향에 대한 반발은 2배가 아니라
        더 크게(반비례이므로) 증폭된다.

        두 번째 로봇(인덱스 1, 호출 규약상 항상 Blocker --
        `herding_core.py`가 `[robot1_pos, robot2_pos]` 순서로 넘긴다)이
        `robot_repulsion_activation_distance_m`보다 멀면 그 로봇의 기여를
        건너뛴다. 첫 번째 로봇(Driver)은 이 게이팅의 영향을 받지 않는다 --
        Driver는 표적을 실제로 추격하는 로봇이라 즉각적인 반발이 항상
        타당하지만, Blocker는 몰이 초반 표적과 멀리 떨어진 대기 지점에서
        시작하는데도 그 거리와 무관하게 항상 반발항에 반영돼 표적의 초반
        도주 경로를 미묘하게 바꿔놓는다는 가설(트러블슈팅 노트 11-9)을
        검증하기 위한 게이팅이다.
        """
        contribution = np.zeros(8)
        for i, robot_pos in enumerate(robot_positions):
            away = target_pos - robot_pos
            dist = np.linalg.norm(away)
            if dist < 1e-6:
                continue
            if i == 1 and dist > self.config.robot_repulsion_activation_distance_m:
                continue
            away = away / dist
            weight = self.config.robot_repulsion_weight / dist
            contribution += np.clip(_DIRECTIONS @ away, 0.0, None) * weight
        return contribution
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd herding_controller && python3 -m pytest test/test_escape_model.py -v`
Expected: 전체 PASS(기존 5개 + 신규 3개 = 8개).

- [ ] **Step 5: 전체 유닛테스트 회귀 확인**

Run: `cd herding_controller && python3 -m pytest test/ -x -q`
Expected: 전부 PASS, 실패 0건(신규 필드가 기본값 `inf`라 다른 어떤 테스트의
동작도 바뀌지 않아야 한다).

- [ ] **Step 6: 커밋**

```bash
git add herding_controller/herding_controller/escape_model.py herding_controller/test/test_escape_model.py
git commit -m "feat(herding): escape_model 로봇 반발항에 Blocker 거리 게이팅 추가

트러블슈팅 노트 11-9/11-10/12 참고 -- Blocker(robot_positions[1])가
robot_repulsion_activation_distance_m보다 멀면 반발항 계산에서 제외한다.
기본값 inf로 기존 동작과 완전히 동일(하위호환)."
```

---

### Task 2: `HerdingConfig`에 필드 추가 + `HerdingCore` 배선

**Files:**
- Modify: `herding_controller/herding_controller/herding_core.py:29-85` (`HerdingConfig`), `herding_core.py:168-174` (`EscapeModelConfig` 생성부)
- Test: `herding_controller/test/test_herding_core.py`

**Interfaces:**
- Consumes: Task 1의 `EscapeModelConfig.robot_repulsion_activation_distance_m`
- Produces: `HerdingConfig.robot_repulsion_activation_distance_m: float`
  (기본값 `float("inf")`) — Task 3의 어블레이션 스크립트가
  `dataclasses.replace(config, robot_repulsion_activation_distance_m=...)`로
  오버라이드할 때 씀.

- [ ] **Step 1: 실패하는 테스트 작성**

`herding_controller/test/test_herding_core.py` 파일 끝에 추가:

```python
def test_robot_repulsion_activation_distance_defaults_to_unbounded():
    core = HerdingCore(make_config())
    assert core.escape_model.config.robot_repulsion_activation_distance_m == float("inf")


def test_robot_repulsion_activation_distance_wired_to_escape_model():
    core = HerdingCore(make_config(robot_repulsion_activation_distance_m=1.0))
    assert core.escape_model.config.robot_repulsion_activation_distance_m == 1.0
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd herding_controller && python3 -m pytest test/test_herding_core.py -v -k robot_repulsion_activation_distance`
Expected: 첫 번째 테스트는 `AttributeError`(`EscapeModelConfig`에 해당 속성
없음), 두 번째 테스트는 `TypeError: HerdingConfig.__init__() got an
unexpected keyword argument`로 FAIL.

- [ ] **Step 3: 최소 구현 작성**

`herding_core.py`의 `HerdingConfig` 안, `deadlock_release_distance_m` 필드
(85번째 줄) 바로 뒤에 추가:

```python
    # Blocker(로봇 2)의 escape_model 반발항 활성화 거리(m) -- 트러블슈팅
    # 노트 11-9/11-10/12 참고. 표적이 Blocker로부터 이 거리보다 멀면
    # Blocker의 존재를 EscapeModel._robot_repulsion() 계산에서 아예
    # 제외한다. 기본값 inf는 게이팅 없음(기존 동작과 완전히 동일) --
    # 최적값은 experiments/blocker_contribution_ablation.py 스윕으로 정한다.
    robot_repulsion_activation_distance_m: float = float("inf")
```

`herding_core.py:168-174`의 `EscapeModelConfig(...)` 생성부를 아래로 교체:

```python
        self.escape_model = EscapeModel(EscapeModelConfig(
            wall_follow_p=config.markov_wall_follow_p, wall_hug_p=config.markov_wall_hug_p,
            center_p=config.markov_center_p, momentum_weight=config.momentum_weight,
            robot_repulsion_weight=config.robot_repulsion_weight,
            wall_detect_radius_cells=config.wall_detect_radius_cells,
            escape_route_top_k=config.escape_route_top_k,
            robot_repulsion_activation_distance_m=config.robot_repulsion_activation_distance_m,
        ), self.grid_map)
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd herding_controller && python3 -m pytest test/test_herding_core.py -v -k robot_repulsion_activation_distance`
Expected: 2개 다 PASS.

- [ ] **Step 5: 전체 유닛테스트 + 검증 스위트 스모크 회귀 확인**

Run: `cd herding_controller && python3 -m pytest test/ -x -q`
Expected: 전부 PASS(기존 142개+ 신규 3+2=5개, 총 147개+).

`config/herding_params.yaml`을 로드하는 `load_herding_config`가 여전히
동작하는지 확인(신규 필드가 YAML에 없어도 dataclass 기본값으로 채워지는지):

Run: `cd herding_controller && python3 -c "
import sys, os
sys.path.insert(0, os.getcwd())
from test.run_validation import CONFIG_PATH, load_herding_config
cfg = load_herding_config(CONFIG_PATH)
assert cfg.robot_repulsion_activation_distance_m == float('inf')
print('OK:', cfg.robot_repulsion_activation_distance_m)
"`
Expected: `OK: inf` 출력, 에러 없음.

- [ ] **Step 6: 커밋**

```bash
git add herding_controller/herding_controller/herding_core.py herding_controller/test/test_herding_core.py
git commit -m "feat(herding): HerdingConfig에 robot_repulsion_activation_distance_m 배선

EscapeModelConfig로 그대로 전달. 기본값 inf, 하위호환."
```

---

### Task 3: 페어드 rescue/regression 어블레이션 스크립트

**Files:**
- Create: `herding_controller/experiments/blocker_contribution_ablation.py`
- Test: `herding_controller/test/test_blocker_contribution_ablation.py`

**Interfaces:**
- Consumes: Task 2의 `HerdingConfig.robot_repulsion_activation_distance_m`,
  기존 `test.run_validation.{CONFIG_PATH, SIM_CONFIG, load_herding_config,
  make_real_map_config}`, `test.simulator.run_trial_real_map`,
  `test.evasion_models.reactive_flee.ReactiveFlee`,
  `test.evasion_models.juking_flee.JukingFlee`, `test.real_map_arena.{TRAPS,
  ROBOT_B_SPAWN}`, `herding_controller.herding_core.HerdingCore`
- Produces: `run_paired_trap(herding_config_base, trap_name, trap_pos,
  trials, seed_base, args) -> dict`(키: `trap`, `rescue`, `regression`,
  `both_success`, `both_fail`, `p_value`) — 이후 세션에서 스윕을 돌릴 때
  이 함수를 재사용한다.

- [ ] **Step 1: 실패하는 테스트 작성**

`herding_controller/test/test_blocker_contribution_ablation.py` 새로 작성:

```python
# herding_controller/test/test_blocker_contribution_ablation.py
"""blocker_contribution_ablation.py 스모크 테스트 -- 작은 trial 수로
run_paired_trap()이 구조적으로 일관된 결과를 내는지만 빠르게 확인한다.
(전체 스윕(N=150/트랩)은 이 테스트가 커버할 범위가 아니다 -- 그건 다음
세션의 실제 실험 실행이 담당한다.)
"""
import argparse
import os
import sys

_EXPERIMENTS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "experiments"
)
sys.path.insert(0, _EXPERIMENTS_DIR)

from blocker_contribution_ablation import run_paired_trap  # noqa: E402
from test import real_map_arena  # noqa: E402
from test.run_validation import CONFIG_PATH, load_herding_config  # noqa: E402


def _args(**overrides):
    defaults = dict(
        robot_repulsion_activation_distance=None,
        juke_probability=None, juke_duration=0.4,
        juke_angle_min=0.0, juke_angle_max=0.0,
    )
    defaults.update(overrides)
    return argparse.Namespace(**defaults)


def test_run_paired_trap_counts_are_internally_consistent():
    herding_config_base = load_herding_config(CONFIG_PATH)
    trap_name, trap_pos = next(iter(real_map_arena.TRAPS.items()))
    row = run_paired_trap(
        herding_config_base, trap_name, trap_pos, trials=3, seed_base=999_000, args=_args()
    )
    total = row["rescue"] + row["regression"] + row["both_success"] + row["both_fail"]
    assert total == 3
    assert 0.0 <= row["p_value"] <= 1.0


def test_run_paired_trap_accepts_activation_distance_override():
    herding_config_base = load_herding_config(CONFIG_PATH)
    trap_name, trap_pos = next(iter(real_map_arena.TRAPS.items()))
    row = run_paired_trap(
        herding_config_base, trap_name, trap_pos, trials=3, seed_base=999_100,
        args=_args(robot_repulsion_activation_distance=1.0),
    )
    total = row["rescue"] + row["regression"] + row["both_success"] + row["both_fail"]
    assert total == 3
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd herding_controller && python3 -m pytest test/test_blocker_contribution_ablation.py -v`
Expected: `ModuleNotFoundError: No module named 'blocker_contribution_ablation'`로 FAIL.

- [ ] **Step 3: 스크립트 구현 작성**

`herding_controller/experiments/blocker_contribution_ablation.py` 새로 작성:

```python
# herding_controller/experiments/blocker_contribution_ablation.py
"""로봇 B(Blocker) 기여도 페어드 rescue/regression 어블레이션 스크립트.

트러블슈팅 노트 11-9에서 처음 쓰였지만 저장소에 커밋되지 않아 이후 세션에서
재사용할 수 없었던 분석 방식을 다시 구현하고, 이번엔 반드시 커밋해서 남긴다.

`HerdingCore._stabilize_blocking_point`를 몽키패치해 Blocker를 스폰 위치에
완전히 고정한 "frozen" 조건과, 원래 알고리즘 그대로인 "active" 조건을
**동일 시드**로 짝지어 실행한 뒤,
- rescue: frozen 실패 -> active 성공
- regression: frozen 성공 -> active 실패
를 트랩별로 센다. 집계 성공률 차이만 보면 서로 상쇄되는 rescue/regression을
놓칠 수 있어서(11-3 vs 11-9), 반드시 시드 단위로 페어링해서 본다.

`--robot-repulsion-activation-distance`로 EscapeModel._robot_repulsion()의
Blocker 게이팅 임계값(m)을 오버라이드할 수 있다(기본값은 HerdingConfig
기본값 그대로, 즉 게이팅 없음). `--juke-probability`를 주면 reactive_flee
대신 그 파라미터로 튜닝한 JukingFlee를 evasion model로 쓴다(강화 시나리오
진단용, 트러블슈팅 노트 12번 항목 참고).

사용법:
    python3 blocker_contribution_ablation.py [--trials-per-trap N] \
        [--robot-repulsion-activation-distance D] \
        [--juke-probability P] [--juke-duration S] \
        [--juke-angle-min A] [--juke-angle-max A]
"""
import argparse
import dataclasses
import os
import sys
from contextlib import contextmanager

import numpy as np
from scipy.stats import binomtest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from herding_controller.herding_core import HerdingCore  # noqa: E402
from test import real_map_arena  # noqa: E402
from test.evasion_models.juking_flee import JukingFlee  # noqa: E402
from test.evasion_models.reactive_flee import ReactiveFlee  # noqa: E402
from test.run_validation import (  # noqa: E402
    CONFIG_PATH,
    SIM_CONFIG,
    load_herding_config,
    make_real_map_config,
)
from test.simulator import run_trial_real_map  # noqa: E402


@contextmanager
def _frozen_blocker():
    """HerdingCore._stabilize_blocking_point를 패치해 Blocker를 스폰 위치에 고정한다.

    패치된 함수는 (candidate, now_sec)를 무시하고 항상
    real_map_arena.ROBOT_B_SPAWN을 돌려주므로, resolve_separation()을
    거쳐도 로봇 2의 목표는 사실상 자기 스폰 위치 그대로다 -- "로봇 B가
    아예 관여하지 않았다면"을 근사한다(트러블슈팅 노트 11-3/11-9의 몽키패치
    방식 재구현).
    """
    original = HerdingCore._stabilize_blocking_point

    def frozen(self, candidate, now_sec):
        return real_map_arena.ROBOT_B_SPAWN.copy()

    HerdingCore._stabilize_blocking_point = frozen
    try:
        yield
    finally:
        HerdingCore._stabilize_blocking_point = original


def _make_evasion_model(herding_config, seed, args):
    rng = np.random.default_rng([seed, 4242])
    speed = SIM_CONFIG.target_max_speed_mps
    if args.juke_probability is None:
        return ReactiveFlee(speed, herding_config.flee_reaction_distance_m)
    return JukingFlee(
        speed, herding_config.flee_reaction_distance_m,
        juke_probability_per_sec=args.juke_probability,
        juke_duration_sec=args.juke_duration,
        juke_angle_range=(args.juke_angle_min, args.juke_angle_max),
        rng=rng,
    )


def run_paired_trap(herding_config_base, trap_name, trap_pos, trials, seed_base, args):
    """한 트랩에서 능동/frozen Blocker를 동일 시드로 페어링해 rescue/regression을 센다."""
    config = make_real_map_config(herding_config_base, trap_pos)
    if args.robot_repulsion_activation_distance is not None:
        config = dataclasses.replace(
            config, robot_repulsion_activation_distance_m=args.robot_repulsion_activation_distance,
        )

    active_results = []
    for i in range(trials):
        seed = seed_base + i
        model = _make_evasion_model(config, seed, args)
        active_results.append(run_trial_real_map(config, model, seed, SIM_CONFIG))

    frozen_results = []
    with _frozen_blocker():
        for i in range(trials):
            seed = seed_base + i
            model = _make_evasion_model(config, seed, args)
            frozen_results.append(run_trial_real_map(config, model, seed, SIM_CONFIG))

    rescue = sum(1 for a, f in zip(active_results, frozen_results) if a.success and not f.success)
    regression = sum(1 for a, f in zip(active_results, frozen_results) if f.success and not a.success)
    both_success = sum(1 for a, f in zip(active_results, frozen_results) if a.success and f.success)
    both_fail = sum(1 for a, f in zip(active_results, frozen_results) if not a.success and not f.success)

    n_discordant = rescue + regression
    p_value = (
        binomtest(min(rescue, regression), n_discordant, 0.5, alternative="two-sided").pvalue
        if n_discordant > 0 else 1.0
    )
    return {
        "trap": trap_name, "rescue": rescue, "regression": regression,
        "both_success": both_success, "both_fail": both_fail, "p_value": p_value,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials-per-trap", type=int, default=150)
    parser.add_argument("--seed-base", type=int, default=1_200_000)
    parser.add_argument("--robot-repulsion-activation-distance", type=float, default=None)
    parser.add_argument("--juke-probability", type=float, default=None)
    parser.add_argument("--juke-duration", type=float, default=0.4)
    parser.add_argument("--juke-angle-min", type=float, default=np.pi / 4)
    parser.add_argument("--juke-angle-max", type=float, default=np.pi / 2)
    args = parser.parse_args()

    herding_config_base = load_herding_config(CONFIG_PATH)

    rows = []
    for offset, (trap_name, trap_pos) in enumerate(real_map_arena.TRAPS.items()):
        row = run_paired_trap(
            herding_config_base, trap_name, trap_pos, args.trials_per_trap,
            args.seed_base + offset * 100_000, args,
        )
        rows.append(row)
        print(f"{trap_name:8s}: rescue={row['rescue']:3d} regression={row['regression']:3d} "
              f"both_success={row['both_success']:3d} both_fail={row['both_fail']:3d} "
              f"p={row['p_value']:.3f}")

    total_rescue = sum(r["rescue"] for r in rows)
    total_regression = sum(r["regression"] for r in rows)
    total_n = args.trials_per_trap * len(rows)
    total_discordant = total_rescue + total_regression
    total_p = (
        binomtest(min(total_rescue, total_regression), total_discordant, 0.5,
                  alternative="two-sided").pvalue
        if total_discordant > 0 else 1.0
    )
    print(f"{'전체':8s}: rescue={total_rescue:3d} regression={total_regression:3d} "
          f"(n={total_n}) p={total_p:.3f}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd herding_controller && python3 -m pytest test/test_blocker_contribution_ablation.py -v`
Expected: 2개 다 PASS. (각 트랩당 active 3회 + frozen 3회 = 6번의
`run_trial_real_map` 호출뿐이라 수 초~수십 초 내 완료되어야 한다.)

- [ ] **Step 5: 스크립트 자체를 CLI로 한 번 더 스모크 실행**

Run: `cd herding_controller && python3 experiments/blocker_contribution_ablation.py --trials-per-trap 3`
Expected: 에러 없이 종료, `top`/`left`/`bottom` 3줄 + `전체` 1줄 출력. 각 줄의
`rescue+regression+both_success+both_fail`이 3(트랩당 trials)과 일치.

- [ ] **Step 6: 전체 유닛테스트 회귀 확인**

Run: `cd herding_controller && python3 -m pytest test/ -x -q`
Expected: 전부 PASS(총 147개+2개=149개+).

- [ ] **Step 7: 커밋**

```bash
git add herding_controller/experiments/blocker_contribution_ablation.py herding_controller/test/test_blocker_contribution_ablation.py
git commit -m "feat(herding): 로봇 B 페어드 rescue/regression 어블레이션 스크립트 추가

11-9에서 커밋되지 않았던 몽키패치 기반 능동 vs frozen Blocker 비교를
재구현해 저장소에 남긴다. --robot-repulsion-activation-distance로 Task 1/2의
게이팅을, --juke-*로 강화 시나리오(JukingFlee 튜닝)를 스윕할 수 있다."
```

---

## 계획 완료 이후 (이 계획의 범위 밖 — 결과 미확정 연구 활동)

이 계획은 코드(게이팅 로직 + 재사용 가능한 스크립트)를 준비하는 데까지만
다룬다. 다음은 스크립트가 완성된 뒤 실제로 실행해서 결과를 봐야 알 수 있는
연구 활동이라 태스크로 미리 못 박지 않는다(스펙 문서 3-5 실행 순서 참고):

1. `--robot-repulsion-activation-distance` 스윕: `inf`(대조군), 2.0, 1.5,
   1.0, 0.5m — `--trials-per-trap 150`으로 표준 reactive_flee 페어드 비교
2. 가장 나은 값으로 `test/run_validation.py`(N=100/트랩) 전체 회귀 재검증
3. `--juke-probability`/`--juke-duration`/`--juke-angle-*` 스크리닝(N=30,
   frozen 조건만) — Driver 단독 성공률이 뚜렷이 낮아지는 조합 탐색
4. 확정된 강화 시나리오 + 1번의 최적 활성화 거리로 다시 N=150 페어드 비교
5. 결과를 `herding_controller_트러블슈팅_노트.md`에 `## 12. 로봇 B 기여도 —
   7차 시도` 섹션으로 기록(성공/실패 무관, 스펙 문서 3-6 참고), 11-7 "다음
   단계 후보" 갱신
