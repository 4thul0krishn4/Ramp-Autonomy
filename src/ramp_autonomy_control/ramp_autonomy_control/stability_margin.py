"""Dependency-free planar stability-margin calculation."""
import math


def compute_pitch_margin(
    pitch_rad: float,
    payload_mass: float,
    payload_cg: float,
    wheelbase: float,
    base_mass: float,
    base_height: float,
) -> float:
    """Return the normalized planar tip-over margin in the range [0, 1]."""
    total_mass = base_mass + payload_mass
    if total_mass <= 0:
        return 1.0

    # The base CG is centered; only the payload shifts the combined CG.
    combined_cg_x = (payload_mass * payload_cg) / total_mass
    half_wheelbase = wheelbase / 2.0

    # Approximate the payload as raising the combined CG by 0.15 m.
    cg_height = base_height / 2.0 + (0.15 if payload_mass > 0 else 0.0)
    pitch_induced_shift = cg_height * math.tan(abs(pitch_rad))
    effective_cg_x = abs(combined_cg_x) + pitch_induced_shift

    margin = 1.0 - (effective_cg_x / half_wheelbase)
    return max(0.0, min(1.0, margin))
