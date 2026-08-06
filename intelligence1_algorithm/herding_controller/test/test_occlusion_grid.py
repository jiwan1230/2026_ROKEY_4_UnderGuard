# herding_controller/test/test_occlusion_grid.py
import numpy as np

from herding_controller.grid_map import GridConfig, GridMap
from herding_controller.occlusion_grid import OcclusionGrid, OcclusionGridConfig


def make_grid_and_belief():
    grid = GridMap(GridConfig(resolution_m=0.25, width_cells=40, height_cells=40))
    belief = OcclusionGrid(OcclusionGridConfig(diffusion_rate=0.2, decay_factor=0.9), grid)
    return grid, belief


def test_seed_sets_best_guess_to_seeded_cell():
    _, belief = make_grid_and_belief()
    belief.seed(10, 10)
    assert belief.best_guess_cell() == (10, 10)


def test_step_diffuses_probability_to_neighbors():
    _, belief = make_grid_and_belief()
    belief.seed(10, 10)
    belief.step(dt=0.1)
    assert belief.belief[10, 11] > 0
    assert belief.belief[10, 9] > 0


def test_step_decays_total_probability_mass():
    _, belief = make_grid_and_belief()
    belief.seed(10, 10)
    total_before = belief.belief.sum()
    belief.step(dt=0.1)
    assert belief.belief.sum() < total_before


def test_obstacle_cells_never_hold_belief():
    grid, belief = make_grid_and_belief()
    grid.obstacle_mask[10, 11] = True
    belief.seed(10, 10)
    belief.step(dt=0.5)
    assert belief.belief[10, 11] == 0.0


def test_repeated_steps_decay_mass_geometrically_not_plateau():
    # 회귀 테스트: step()은 예전에 decay_factor를 적용하기 전에 확산된 belief를
    # 다시 합계 1.0으로 재정규화했는데, 이는 이전 스텝의 decay를 모두 지워버려서
    # 타겟을 관측하지 않은 채 스텝이 진행될수록 기하급수적으로(decay_factor ** n)
    # 감소하는 대신 총 질량이 정확히 decay_factor에 영원히 고정되게 만들었다.
    _, belief = make_grid_and_belief()
    belief.seed(10, 10)
    for _ in range(50):
        belief.step(dt=0.1)
    # decay_factor=0.9로 50 스텝 -> 0.9**50 ~= 0.00515; 정체된(plateau)
    # 구현이라면 대신 정확히 0.9에 영원히 머물러 있을 것이다.
    assert belief.belief.sum() < 0.9 ** 10
    assert belief.belief.sum() >= 0.0


def test_large_dt_does_not_inflate_belief_mass():
    # 회귀 테스트: diffusion_rate=0.2, dt=5.0일 때, 클램핑되지 않은 확산 가중치는
    # 1.0이 되어(4*weight=4.0 >> 1), "keep" 항(1 - 4*weight)이 크게 음수가 된다.
    # 확산 전에 가중치 자체를 클램핑하는 대신 확산 후에 음수 결과를 0으로 클리핑하는
    # 단순한 구현은 스텝 이전보다 총 belief 질량을 부풀린다: 예를 들어 weight=0.3일 때,
    # 클리핑 전 점 질량 1.0은 center=-0.2, 4개의 이웃=+0.3씩(총합 1.0으로 보존됨)이
    # 되지만, center의 -0.2를 0으로 클리핑하면 총합이 1.2가 된다 -- decay_factor가
    # 적용되기도 전에 스텝 이전보다 질량이 더 많아진 것이다. 확산은 절대 총 질량을
    # 증가시켜서는 안 된다; 그렇지 않음을 검증한다.
    _, belief = make_grid_and_belief()
    belief.seed(20, 20)
    total_before = belief.belief.sum()
    belief.step(dt=5.0)
    # decay 적용 전의 확산된 총합과 비교한다(decay_factor < 1이므로
    # decay 이후 총합보다 크거나 같음). 이렇게 하면 확산의 질량 보존을
    # (예상되며 별도로 테스트되는) decay 감소와 분리하여 검증할 수 있다.
    total_after_predecay = belief.belief.sum() / belief.config.decay_factor
    assert total_after_predecay <= total_before + 1e-9
