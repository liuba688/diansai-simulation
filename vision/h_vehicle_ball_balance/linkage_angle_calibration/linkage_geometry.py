"""Geometry helpers for the 2026 H ball-beam linkage calibration."""

import math


CRANK_RADIUS_MM = 35.0
CONNECTING_ROD_MM = 55.5
BEAM_LENGTH_MM = 253.0

# Existing X42S bench mapping: 160 position pulses correspond to 18 motor deg.
MOTOR_DEG_PER_PULSE = 18.0 / 160.0


def motor_deg_from_pulses(pulses):
    return float(pulses) * MOTOR_DEG_PER_PULSE


def theoretical_beam_deg_from_pulses(pulses):
    """Return the small-motion 9-o'clock linkage estimate.

    With the crank horizontal at zero, crank-pin vertical displacement is
    r*sin(theta). The actual installed linkage must still be measured because
    joint offsets, clearance and beam attachment geometry are not modeled.
    """

    motor_rad = math.radians(motor_deg_from_pulses(pulses))
    free_end_height_mm = CRANK_RADIUS_MM * math.sin(motor_rad)
    return math.degrees(math.atan2(free_end_height_mm, BEAM_LENGTH_MM))


def geometry_summary():
    return "r={:.1f} l={:.1f} L={:.1f} mm".format(
        CRANK_RADIUS_MM,
        CONNECTING_ROD_MM,
        BEAM_LENGTH_MM,
    )
