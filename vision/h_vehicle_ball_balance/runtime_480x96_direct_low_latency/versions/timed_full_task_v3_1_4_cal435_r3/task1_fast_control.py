"""Model-feedforward controller for the complete static ball task."""

import math

from center_control import (
    BallEstimator,
    CENTER_0_PX,
    NEGATIVE_50_PX,
    POSITIVE_50_PX,
    beam_angle_to_motor_pulses,
)


CONTROL_PROFILE_NAME = "model_quintic_task1_v2_0_1"

ARM_CENTER_TOLERANCE_MM = 8.0
ARM_MAX_SPEED_MM_S = 20.0
ARM_STABLE_TIME_S = 0.35
CENTER_HOLD_TIME_S = 0.25
BALL_EDGE_LIMIT_MM = 105.0

POSITIVE_TARGET_MM = 50.0
NEGATIVE_TARGET_MM = -50.0
POSITIVE_MOVE_TIME_S = 1.25
NEGATIVE_MOVE_TIME_S = 1.75
POSITIVE_SETTLE_ERROR_MM = 8.0
POSITIVE_SETTLE_SPEED_MM_S = 25.0
POSITIVE_SETTLE_TIME_S = 0.12
POSITIVE_SETTLE_TIMEOUT_S = 1.00
FINAL_SETTLE_ERROR_MM = 8.0
FINAL_SETTLE_SPEED_MM_S = 12.0
FINAL_SETTLE_TIME_S = 0.25
TASK_TIME_LIMIT_S = 5.00

# Solid-sphere rolling model: a = (5/7) * g * sin(theta).
GRAVITY_M_S2 = 9.80665
ROLLING_ACCELERATION_FACTOR = 5.0 / 7.0
MAX_BEAM_ANGLE_DEG = 3.00

# Acceleration-domain state feedback. The quintic trajectory supplies the
# nominal acceleration; feedback only rejects model and friction errors.
MOVE_POSITION_ACCEL_KP_PER_S2 = 4.0
MOVE_VELOCITY_ACCEL_KD_PER_S = 1.4
HOLD_POSITION_ACCEL_KP_PER_S2 = 8.0
HOLD_VELOCITY_ACCEL_KD_PER_S = 3.0
ROLLING_FRICTION_FF_MM_S2 = 35.0
FRICTION_FF_MIN_REFERENCE_SPEED_MM_S = 10.0
POSITIVE_DRIVE_ANGLE_LIMIT_DEG = 2.30
NEGATIVE_DRIVE_ANGLE_LIMIT_DEG = 2.40

# Explicit breakaway fallback for measured static friction. Positive and
# negative values remain independent because the linkage is asymmetric.
STICTION_TRIGGER_POSITION_ERROR_MM = 5.0
STICTION_TRIGGER_SPEED_MM_S = 6.0
STICTION_TRIGGER_TIME_S = 0.10
STICTION_RELEASE_SPEED_MM_S = 15.0
POSITIVE_STICTION_MIN_ANGLE_DEG = 1.05
NEGATIVE_STICTION_MIN_ANGLE_DEG = 1.15


def _clamp(value, low, high):
    return max(low, min(high, value))


def _sign(value):
    if value > 0.0:
        return 1.0
    if value < 0.0:
        return -1.0
    return 0.0


def quintic_reference(start_mm, end_mm, duration_s, elapsed_s):
    """Return position, velocity and acceleration for a minimum-jerk move."""

    duration_s = max(0.001, float(duration_s))
    tau = _clamp(float(elapsed_s) / duration_s, 0.0, 1.0)
    tau2 = tau * tau
    tau3 = tau2 * tau
    tau4 = tau3 * tau
    tau5 = tau4 * tau
    displacement_mm = float(end_mm) - float(start_mm)

    blend = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5
    blend_d = 30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4
    blend_dd = 60.0 * tau - 180.0 * tau2 + 120.0 * tau3

    position_mm = float(start_mm) + displacement_mm * blend
    velocity_mm_s = displacement_mm * blend_d / duration_s
    acceleration_mm_s2 = (
        displacement_mm * blend_dd / (duration_s * duration_s)
    )
    if elapsed_s >= duration_s:
        return float(end_mm), 0.0, 0.0
    return position_mm, velocity_mm_s, acceleration_mm_s2


def acceleration_to_beam_angle_deg(acceleration_mm_s2):
    """Invert the rolling-sphere model and enforce the calibrated angle range."""

    normalized = (
        float(acceleration_mm_s2)
        * 0.001
        / (ROLLING_ACCELERATION_FACTOR * GRAVITY_M_S2)
    )
    maximum_sine = math.sin(math.radians(MAX_BEAM_ANGLE_DEG))
    normalized = _clamp(normalized, -maximum_sine, maximum_sine)
    return math.degrees(math.asin(normalized))


class BallTaskController:
    """Execute O -> +50 mm -> -50 mm with a five-second success budget."""

    def __init__(self, pixel_calibration_only=False):
        self.estimator = BallEstimator()
        self.pixel_calibration_only = bool(pixel_calibration_only)
        self.state = (
            "PIXEL_CALIBRATION"
            if self.pixel_calibration_only
            else "WAIT_CENTER"
        )
        self.fault_reason = ""
        self.arm_since = None
        self.state_since = None
        self.settle_since = None
        self.start_time = None
        self.last_update_time = None

        self.move_start_time = None
        self.move_start_mm = 0.0
        self.move_end_mm = 0.0
        self.move_duration_s = 1.0
        self.motion_direction = 0.0

        self.target_mm = 0.0
        self.target_goal_mm = 0.0
        self.reference_velocity_mm_s = 0.0
        self.reference_acceleration_mm_s2 = 0.0
        self.desired_acceleration_mm_s2 = 0.0
        self.requested_beam_angle_deg = 0.0
        self.motor_target_pulses = 0
        self.active_drive_limit_pulses = beam_angle_to_motor_pulses(
            MAX_BEAM_ANGLE_DEG
        )

        self.stiction_active = False
        self.stiction_stationary_time_s = 0.0
        self.stiction_event_count = 0

    def force_fault(self, reason):
        self.state = "FAULT"
        self.fault_reason = str(reason)
        self.motion_direction = 0.0
        self.reference_velocity_mm_s = 0.0
        self.reference_acceleration_mm_s2 = 0.0
        self.desired_acceleration_mm_s2 = 0.0
        self.requested_beam_angle_deg = 0.0
        self.motor_target_pulses = 0
        self.stiction_active = False
        self.stiction_stationary_time_s = 0.0
        self.active_drive_limit_pulses = 0

    def _begin_move(self, state, start_mm, end_mm, duration_s, now):
        self.state = str(state)
        self.state_since = now
        self.move_start_time = now
        self.move_start_mm = float(start_mm)
        self.move_end_mm = float(end_mm)
        self.move_duration_s = float(duration_s)
        self.motion_direction = _sign(self.move_end_mm - self.move_start_mm)
        self.target_goal_mm = self.move_end_mm
        self.settle_since = None
        self.stiction_active = False
        self.stiction_stationary_time_s = 0.0

    def _update_state(self, position_mm, velocity_mm_s, valid, now):
        if self.state in ("FAULT", "PIXEL_CALIBRATION", "COMPLETE"):
            return

        if self.state == "WAIT_CENTER":
            if (
                valid
                and abs(position_mm) <= ARM_CENTER_TOLERANCE_MM
                and abs(velocity_mm_s) <= ARM_MAX_SPEED_MM_S
            ):
                if self.arm_since is None:
                    self.arm_since = now
                elif now - self.arm_since >= ARM_STABLE_TIME_S:
                    self.state = "HOLD_CENTER"
                    self.start_time = now
                    self.state_since = now
                    self.target_goal_mm = 0.0
            else:
                self.arm_since = None
            return

        if not valid:
            self.force_fault("vision timeout")
            return
        if abs(position_mm) >= BALL_EDGE_LIMIT_MM:
            self.force_fault("ball edge")
            return
        if (
            self.start_time is not None
            and now - self.start_time >= TASK_TIME_LIMIT_S
        ):
            self.force_fault("task timeout")
            return

        if self.state == "HOLD_CENTER":
            self.target_goal_mm = 0.0
            if now - self.state_since >= CENTER_HOLD_TIME_S:
                self._begin_move(
                    "MOVE_POSITIVE",
                    position_mm,
                    POSITIVE_TARGET_MM,
                    POSITIVE_MOVE_TIME_S,
                    now,
                )
            return

        if self.state == "MOVE_POSITIVE":
            if now - self.move_start_time >= self.move_duration_s:
                self.state = "HOLD_POSITIVE"
                self.state_since = now
                self.target_goal_mm = POSITIVE_TARGET_MM
                self.motion_direction = 1.0
                self.settle_since = None
            return

        if self.state == "HOLD_POSITIVE":
            self.target_goal_mm = POSITIVE_TARGET_MM
            if (
                abs(position_mm - POSITIVE_TARGET_MM)
                <= POSITIVE_SETTLE_ERROR_MM
                and abs(velocity_mm_s) <= POSITIVE_SETTLE_SPEED_MM_S
            ):
                if self.settle_since is None:
                    self.settle_since = now
                elif now - self.settle_since >= POSITIVE_SETTLE_TIME_S:
                    self._begin_move(
                        "MOVE_NEGATIVE",
                        position_mm,
                        NEGATIVE_TARGET_MM,
                        NEGATIVE_MOVE_TIME_S,
                        now,
                    )
            else:
                self.settle_since = None

            if (
                self.state == "HOLD_POSITIVE"
                and now - self.state_since >= POSITIVE_SETTLE_TIMEOUT_S
            ):
                self.force_fault("positive settle")
            return

        if self.state == "MOVE_NEGATIVE":
            if now - self.move_start_time >= self.move_duration_s:
                self.state = "HOLD_NEGATIVE"
                self.state_since = now
                self.target_goal_mm = NEGATIVE_TARGET_MM
                self.motion_direction = -1.0
                self.settle_since = None
            return

        if self.state == "HOLD_NEGATIVE":
            self.target_goal_mm = NEGATIVE_TARGET_MM
            if (
                abs(position_mm - NEGATIVE_TARGET_MM)
                <= FINAL_SETTLE_ERROR_MM
                and abs(velocity_mm_s) <= FINAL_SETTLE_SPEED_MM_S
            ):
                if self.settle_since is None:
                    self.settle_since = now
                elif now - self.settle_since >= FINAL_SETTLE_TIME_S:
                    self.state = "COMPLETE"
                    self.state_since = now
                    self.motion_direction = 0.0
                    self.stiction_active = False
                    self.stiction_stationary_time_s = 0.0
            else:
                self.settle_since = None

    def _update_reference(self, now):
        if self.state in ("MOVE_POSITIVE", "MOVE_NEGATIVE"):
            (
                self.target_mm,
                self.reference_velocity_mm_s,
                self.reference_acceleration_mm_s2,
            ) = quintic_reference(
                self.move_start_mm,
                self.move_end_mm,
                self.move_duration_s,
                now - self.move_start_time,
            )
            return

        if self.state in ("HOLD_POSITIVE",):
            self.target_mm = POSITIVE_TARGET_MM
        elif self.state in ("HOLD_NEGATIVE", "COMPLETE"):
            self.target_mm = NEGATIVE_TARGET_MM
        else:
            self.target_mm = 0.0
        self.reference_velocity_mm_s = 0.0
        self.reference_acceleration_mm_s2 = 0.0

    def _apply_stiction_compensation(
        self,
        requested_angle_deg,
        position_mm,
        velocity_mm_s,
        dt,
    ):
        direction = self.motion_direction
        goal_error_mm = self.target_goal_mm - position_mm
        stiction_minimum_angle_deg = (
            POSITIVE_STICTION_MIN_ANGLE_DEG
            if direction > 0.0
            else NEGATIVE_STICTION_MIN_ANGLE_DEG
        )
        eligible = (
            direction != 0.0
            and direction * goal_error_mm
            >= STICTION_TRIGGER_POSITION_ERROR_MM
            and direction * requested_angle_deg > 0.0
            and abs(requested_angle_deg) < stiction_minimum_angle_deg
        )

        if self.stiction_active:
            if (
                not eligible
                or direction * velocity_mm_s
                >= STICTION_RELEASE_SPEED_MM_S
            ):
                self.stiction_active = False
                self.stiction_stationary_time_s = 0.0
        elif (
            eligible
            and abs(velocity_mm_s) <= STICTION_TRIGGER_SPEED_MM_S
        ):
            self.stiction_stationary_time_s += dt
            if self.stiction_stationary_time_s >= STICTION_TRIGGER_TIME_S:
                self.stiction_active = True
                self.stiction_event_count += 1
        else:
            self.stiction_stationary_time_s = 0.0

        if not self.stiction_active:
            return requested_angle_deg
        if direction > 0.0:
            return max(requested_angle_deg, POSITIVE_STICTION_MIN_ANGLE_DEG)
        return min(requested_angle_deg, -NEGATIVE_STICTION_MIN_ANGLE_DEG)

    def update(self, measured_pixel_x, confidence, now):
        if measured_pixel_x is not None:
            self.estimator.update(measured_pixel_x, confidence, now)

        if self.last_update_time is None:
            dt = 0.01
        else:
            dt = _clamp(now - self.last_update_time, 0.001, 0.100)
        self.last_update_time = now

        position_mm, velocity_mm_s, age_s, valid = (
            self.estimator.estimate(now)
        )
        self._update_state(position_mm, velocity_mm_s, valid, now)
        self._update_reference(now)

        if self.state not in (
            "WAIT_CENTER",
            "PIXEL_CALIBRATION",
            "FAULT",
        ) and valid:
            position_error_mm = self.target_mm - position_mm
            velocity_error_mm_s = (
                self.reference_velocity_mm_s - velocity_mm_s
            )
            if self.state in ("MOVE_POSITIVE", "MOVE_NEGATIVE"):
                position_gain = MOVE_POSITION_ACCEL_KP_PER_S2
                velocity_gain = MOVE_VELOCITY_ACCEL_KD_PER_S
            else:
                position_gain = HOLD_POSITION_ACCEL_KP_PER_S2
                velocity_gain = HOLD_VELOCITY_ACCEL_KD_PER_S
            friction_acceleration_mm_s2 = 0.0
            if (
                abs(self.reference_velocity_mm_s)
                >= FRICTION_FF_MIN_REFERENCE_SPEED_MM_S
            ):
                friction_acceleration_mm_s2 = (
                    ROLLING_FRICTION_FF_MM_S2
                    * _sign(self.reference_velocity_mm_s)
                )

            desired_acceleration_mm_s2 = (
                self.reference_acceleration_mm_s2
                + friction_acceleration_mm_s2
                + position_gain * position_error_mm
                + velocity_gain * velocity_error_mm_s
            )
            requested_angle_deg = acceleration_to_beam_angle_deg(
                desired_acceleration_mm_s2
            )
            if self.motion_direction > 0.0:
                requested_angle_deg = min(
                    requested_angle_deg,
                    POSITIVE_DRIVE_ANGLE_LIMIT_DEG,
                )
            elif self.motion_direction < 0.0:
                requested_angle_deg = max(
                    requested_angle_deg,
                    -NEGATIVE_DRIVE_ANGLE_LIMIT_DEG,
                )
            requested_angle_deg = self._apply_stiction_compensation(
                requested_angle_deg,
                position_mm,
                velocity_mm_s,
                dt,
            )
            self.desired_acceleration_mm_s2 = desired_acceleration_mm_s2
            self.requested_beam_angle_deg = _clamp(
                requested_angle_deg,
                -MAX_BEAM_ANGLE_DEG,
                MAX_BEAM_ANGLE_DEG,
            )
            self.motor_target_pulses = beam_angle_to_motor_pulses(
                self.requested_beam_angle_deg
            )
            self.active_drive_limit_pulses = beam_angle_to_motor_pulses(
                MAX_BEAM_ANGLE_DEG
            )
        else:
            self.desired_acceleration_mm_s2 = 0.0
            self.requested_beam_angle_deg = 0.0
            self.motor_target_pulses = 0
            self.stiction_active = False
            self.stiction_stationary_time_s = 0.0

        elapsed_s = (
            0.0
            if self.start_time is None
            else max(0.0, now - self.start_time)
        )
        return {
            "state": self.state,
            "fault_reason": self.fault_reason,
            "position_mm": position_mm,
            "velocity_mm_s": velocity_mm_s,
            "age_ms": age_s * 1000.0,
            "valid": valid,
            "confidence": self.estimator.confidence,
            "target_mm": self.target_mm,
            "target_goal_mm": self.target_goal_mm,
            "reference_velocity_mm_s": self.reference_velocity_mm_s,
            "reference_acceleration_mm_s2": (
                self.reference_acceleration_mm_s2
            ),
            "desired_acceleration_mm_s2": self.desired_acceleration_mm_s2,
            "requested_beam_angle_deg": self.requested_beam_angle_deg,
            "integral_angle_deg": 0.0,
            "stiction_active": self.stiction_active,
            "stiction_stationary_ms": (
                self.stiction_stationary_time_s * 1000.0
            ),
            "stiction_event_count": self.stiction_event_count,
            "motor_target_pulses": self.motor_target_pulses,
            "drive_limit_pulses": self.active_drive_limit_pulses,
            "elapsed_s": elapsed_s,
        }
