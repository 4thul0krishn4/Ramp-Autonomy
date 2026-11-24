import math

import pytest

from ramp_autonomy_control.stability_margin import compute_pitch_margin


ROBOT_GEOMETRY = {
    'wheelbase': 0.56,
    'base_mass': 40.0,
    'base_height': 0.25,
}


def margin(pitch_deg, payload_mass=0.0, payload_cg=0.0):
    return compute_pitch_margin(
        math.radians(pitch_deg),
        payload_mass,
        payload_cg,
        **ROBOT_GEOMETRY,
    )


def test_flat_unloaded_robot_has_full_margin():
    assert margin(0.0) == 1.0


def test_margin_decreases_monotonically_with_absolute_pitch():
    pitches = [0.0, 5.0, 16.0, 30.0, 45.0, 60.0, 89.999]
    margins = [margin(pitch) for pitch in pitches]
    assert margins == sorted(margins, reverse=True)


@pytest.mark.parametrize('pitch_deg', [0.0, 5.0, 16.0, 45.0, 89.999])
def test_margin_is_symmetric_for_nose_up_and_nose_down(pitch_deg):
    assert margin(pitch_deg, 100.0, 0.1) == margin(-pitch_deg, 100.0, 0.1)


def test_offset_payload_mass_reduces_margin_monotonically():
    masses = [0.0, 25.0, 50.0, 100.0]
    margins = [margin(16.0, mass, 0.1) for mass in masses]
    assert margins == sorted(margins, reverse=True)


@pytest.mark.parametrize('payload_cg', [-1e12, -0.1, 0.0, 0.1, 1e12])
def test_zero_payload_mass_ignores_payload_offset(payload_cg):
    assert margin(16.0, 0.0, payload_cg) == margin(16.0, 0.0, 0.0)


@pytest.mark.parametrize('pitch_deg', [-90.0, -89.999, 0.0, 89.999, 90.0])
@pytest.mark.parametrize('payload_mass', [-1e12, 0.0, 100.0, 1e12])
@pytest.mark.parametrize('payload_cg', [-1e12, 0.0, 1e12])
def test_margin_is_bounded_for_extreme_inputs(pitch_deg, payload_mass, payload_cg):
    result = margin(pitch_deg, payload_mass, payload_cg)
    assert math.isfinite(result)
    assert 0.0 <= result <= 1.0


def test_margin_reaches_zero_as_pitch_approaches_vertical():
    almost_vertical = math.nextafter(math.pi / 2.0, 0.0)
    result = compute_pitch_margin(
        almost_vertical,
        payload_mass=100.0,
        payload_cg=0.1,
        **ROBOT_GEOMETRY,
    )
    assert result == 0.0


def test_published_16_degree_margin_floors():
    assert margin(16.0, 100.0, 0.1) == pytest.approx(0.4632730267419871)
    assert margin(16.0) == pytest.approx(0.8719886670719608)
