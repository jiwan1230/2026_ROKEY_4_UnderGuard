# herding_controller_dual/test/test_run_validation.py
"""검증 하네스(run_validation.py) 자체에 대한 스모크 테스트.

test/ 안의 어떤 것도 run_validation.py를 실행해보지 않았고, 이 때문에 config
불변식 회귀가 `python3 test/run_validation.py 100`(최종 보고서의 재현 명령)을
전체 trial 연산을 다 소모한 후에야 크래시시키는 동안, 127개의 단위 테스트는
계속 통과 상태로 남아 있었다. 이 테스트들은 아주 작은 trial 수로 스윕을
end-to-end로 실행하여 그 경로를 커버한다.
"""
import test.run_validation as run_validation
from herding_controller_dual.herding_core import HerdingConfig
from test.run_validation import (
    CONFIG_PATH,
    SENSITIVITY_SWEEPS,
    _format_sensitivity_cells,
    _run_sensitivity_sweep,
    _write_sensitivity_plot,
    load_herding_config,
)


def test_shipping_sweep_contains_points_the_config_invariant_rejects():
    """스윕이 애초에 왜 rejection 처리를 필요로 하는지를 문서화한다.

    이 스윕은 의도적으로 파라미터 공간에서 교착 상태에 빠지기 쉬운 코너
    (타겟의 반응 반경을 넘어서는 완화된 drive distance)를 탐색하는데, 이는
    정확히 HerdingConfig.__post_init__이 생성을 거부하는 값이다.
    """
    config = load_herding_config(CONFIG_PATH)
    violating = [
        value for value in SENSITIVITY_SWEEPS["drive_distance_m"]
        if value * config.drive_distance_ease_factor >= config.flee_reaction_distance_m
    ]
    assert violating, (
        "no swept drive_distance_m violates the invariant any more -- if the sweep or the "
        "shipping config changed, keep a rejection case covered or drop this handling"
    )


def test_sensitivity_sweep_completes_with_the_shipping_config():
    """이 테스트의 핵심: 불변식을 위반하는 스윕 지점이 실행을 중단시켜서는 안 된다."""
    config = load_herding_config(CONFIG_PATH)
    sweep = _run_sensitivity_sweep(config, trials=1, seed_base=0)  # 예외가 발생하면 안 됨

    assert set(sweep) == set(SENSITIVITY_SWEEPS)
    for param, data in sweep.items():
        assert data["values"] == list(SENSITIVITY_SWEEPS[param])
        # 스윕된 모든 지점은 rejected 여부와 무관하게 여전히 표현된다 -- 조용한 누락이 없음.
        assert len(data["success_rates"]) == len(data["values"])
        for value, rate in zip(data["values"], data["success_rates"]):
            if rate is None:
                assert value in data["rejected"]
            else:
                assert 0.0 <= rate <= 1.0

    drive = sweep["drive_distance_m"]
    assert set(drive["rejected"]) == {0.45, 0.6}
    for reason in drive["rejected"].values():
        assert "flee_reaction_distance_m" in reason
    # 유효한 지점들은 통째로 rejected 되지 않고 실제로 실행되었다.
    assert drive["success_rates"][0] is not None
    assert drive["success_rates"][1] is not None


def test_sweep_over_a_valid_config_rejects_nothing():
    """margin이 충분한 config는 모든 스윕 지점을 실행 가능하게 남겨둔다."""
    config = load_herding_config(CONFIG_PATH)
    roomy = HerdingConfig(**{
        **{f: getattr(config, f) for f in config.__dataclass_fields__},
        "drive_distance_m": 0.5, "drive_distance_ease_factor": 1.0,
        "flee_reaction_distance_m": 3.0,
    })
    sweep = _run_sensitivity_sweep(roomy, trials=1, seed_base=0)
    assert sweep["drive_distance_m"]["rejected"] == {}
    assert all(rate is not None for rate in sweep["drive_distance_m"]["success_rates"])


def _sweep_with_a_rejected_point() -> dict:
    return {
        "drive_distance_m": {
            "values": [0.6, 0.75, 0.9], "success_rates": [0.4, 0.8, None],
            "rejected": {0.9: "drive_distance_m * drive_distance_ease_factor must be < "
                              "flee_reaction_distance_m ..."},
            "baseline": 0.75, "trials": 3,
        },
    }


def test_report_cells_mark_a_rejected_point_instead_of_dropping_it():
    cells = _format_sensitivity_cells(_sweep_with_a_rejected_point()["drive_distance_m"])
    assert "0.75*=80%" in cells       # baseline은 여전히 별표 표시됨
    assert "0.9=REJECTED" in cells    # rejected된 지점이 존재하며 표시됨
    assert "0.6=40%" in cells


def test_sensitivity_plot_renders_a_rejected_point_without_crashing(tmp_path, monkeypatch):
    monkeypatch.setattr(run_validation, "OUTPUT_DIR", str(tmp_path))
    _write_sensitivity_plot(_sweep_with_a_rejected_point())
    assert (tmp_path / "parameter_sensitivity.png").exists()


def test_sensitivity_plot_still_renders_a_fully_valid_sweep(tmp_path, monkeypatch):
    monkeypatch.setattr(run_validation, "OUTPUT_DIR", str(tmp_path))
    data = _sweep_with_a_rejected_point()
    data["drive_distance_m"]["success_rates"] = [0.4, 0.8, 0.1]
    data["drive_distance_m"]["rejected"] = {}
    _write_sensitivity_plot(data)
    assert (tmp_path / "parameter_sensitivity.png").exists()
