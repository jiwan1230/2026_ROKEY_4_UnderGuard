# herding_controller/test/test_run_validation.py
"""Smoke tests for the validation harness itself.

Nothing in test/ exercised run_validation.py, which is how a config-invariant
regression made `python3 test/run_validation.py 100` -- the reproduction command
in the final report -- crash after burning the full trial compute, while 127 unit
tests stayed green. These tests run the sweep end-to-end at a tiny trial count so
that path is covered.
"""
import test.run_validation as run_validation
from herding_controller.herding_core import HerdingConfig
from test.run_validation import (
    CONFIG_PATH,
    SENSITIVITY_SWEEPS,
    _format_sensitivity_cells,
    _run_sensitivity_sweep,
    _write_sensitivity_plot,
    load_herding_config,
)


def test_shipping_sweep_contains_points_the_config_invariant_rejects():
    """Documents *why* the sweep needs rejection handling at all.

    The sweep deliberately probes the deadlock-prone corner of the parameter space
    (eased drive distance beyond the target's reaction radius), which is exactly what
    HerdingConfig.__post_init__ refuses to construct.
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
    """The whole point: an invariant-violating sweep point must not abort the run."""
    config = load_herding_config(CONFIG_PATH)
    sweep = _run_sensitivity_sweep(config, trials=1, seed_base=0)  # must not raise

    assert set(sweep) == set(SENSITIVITY_SWEEPS)
    for param, data in sweep.items():
        assert data["values"] == list(SENSITIVITY_SWEEPS[param])
        # Every swept point is still represented, rejected or not -- no silent gaps.
        assert len(data["success_rates"]) == len(data["values"])
        for value, rate in zip(data["values"], data["success_rates"]):
            if rate is None:
                assert value in data["rejected"]
            else:
                assert 0.0 <= rate <= 1.0

    drive = sweep["drive_distance_m"]
    assert set(drive["rejected"]) == {0.9, 1.05}
    for reason in drive["rejected"].values():
        assert "flee_reaction_distance_m" in reason
    # The valid points really ran rather than being rejected wholesale.
    assert drive["success_rates"][0] is not None
    assert drive["success_rates"][1] is not None


def test_sweep_over_a_valid_config_rejects_nothing():
    """A config with enough margin leaves every sweep point runnable."""
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
    assert "0.75*=80%" in cells       # baseline still starred
    assert "0.9=REJECTED" in cells    # rejected point present and labelled
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
