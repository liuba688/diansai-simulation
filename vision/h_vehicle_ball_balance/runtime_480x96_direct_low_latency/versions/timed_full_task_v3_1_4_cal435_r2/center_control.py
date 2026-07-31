"""Center-hold cascade controller for the calibrated linkage."""


CONTROL_PROFILE_NAME = "cascade_positive_stiction_probe_v2"

POSITIVE_50_PX = 212.0
CENTER_0_PX = 324.0
NEGATIVE_50_PX = 435.0

POSITIVE_50_MM = 50.0
NEGATIVE_50_MM = -50.0

VISION_TIMEOUT_S = 0.150
MAX_PREDICTION_S = 0.050
MAX_ABS_VELOCITY_MM_S = 1000.0
VELOCITY_FILTER_ALPHA = 0.35
OUTLIER_MIN_JUMP_MM = 12.0
OUTLIER_MAX_SPEED_MM_S = 300.0
OUTLIER_REACQUIRE_FRAMES = 2
OUTLIER_REACQUIRE_TOLERANCE_MM = 8.0

ARM_CENTER_TOLERANCE_MM = 12.0
ARM_MAX_SPEED_MM_S = 30.0
ARM_STABLE_TIME_S = 0.40
BALL_EDGE_LIMIT_MM = 105.0
REFERENCE_SLEW_MM_S = 22.0
NEGATIVE_REFERENCE_SLEW_MM_S = 18.0
CENTER_HOLD_BEFORE_STEP_S = 1.0
POSITIVE_TARGET_MM = 50.0
NEGATIVE_TARGET_MM = -50.0
POSITION_SETTLE_ERROR_MM = 3.0
POSITION_SETTLE_SPEED_MM_S = 15.0
POSITION_SETTLE_TIME_S = 0.50
POSITIVE_HOLD_TIME_S = 0.60
STEP_TEST_TIMEOUT_S = 12.0
POSITIVE_ONLY_TEST = True

# Outer position PD: position error/velocity -> requested beam angle.
POSITION_KP_DEG_PER_MM = 0.035
VELOCITY_KD_DEG_PER_MM_S = 0.018
POSITION_KI_DEG_PER_MM_S = 0.012
INTEGRAL_ACTIVE_ERROR_MM = 25.0
INTEGRAL_MAX_SPEED_MM_S = 10.0
INTEGRAL_ANGLE_LIMIT_DEG = 0.50
INTEGRAL_DECAY_PER_S = 1.5
MAX_BEAM_ANGLE_DEG = 1.80
CALIBRATED_MAX_BEAM_ANGLE_DEG = 3.00
ANGLE_DEADBAND_DEG = 0.025

# Positive-direction stiction compensation. This replaces the slow integral
# during TO_POSITIVE so a stop/restart event is explicit in the log.
POSITIVE_STICTION_TRIGGER_ERROR_MM = 8.0
POSITIVE_STICTION_TRIGGER_SPEED_MM_S = 5.0
POSITIVE_STICTION_TRIGGER_TIME_S = 0.12
POSITIVE_STICTION_RELEASE_SPEED_MM_S = 12.0
POSITIVE_STICTION_MIN_ANGLE_DEG = 1.05

# Measured 2026-07-30 linkage map:
# 0 deg -> 0 pulse, 1 deg -> 80 pulse, 3 deg -> 160 pulse.
PULSES_AT_ONE_DEG = 80.0
PULSES_AT_THREE_DEG = 160.0


def _clamp(value, low, high):
    return max(low, min(high, value))


def pixel_to_mm(pixel_x):
    """Piecewise three-point calibration; image-left is positive."""

    pixel_x = float(pixel_x)
    if pixel_x <= CENTER_0_PX:
        return (
            (CENTER_0_PX - pixel_x)
            * POSITIVE_50_MM
            / (CENTER_0_PX - POSITIVE_50_PX)
        )
    return (
        -(pixel_x - CENTER_0_PX)
        * abs(NEGATIVE_50_MM)
        / (NEGATIVE_50_PX - CENTER_0_PX)
    )


def beam_angle_to_motor_pulses(angle_deg):
    """Invert the measured piecewise linkage map."""

    angle_deg = _clamp(
        float(angle_deg),
        -CALIBRATED_MAX_BEAM_ANGLE_DEG,
        CALIBRATED_MAX_BEAM_ANGLE_DEG,
    )
    magnitude = abs(angle_deg)
    if magnitude < ANGLE_DEADBAND_DEG:
        return 0
    if magnitude <= 1.0:
        pulses = PULSES_AT_ONE_DEG * magnitude
    else:
        pulses = PULSES_AT_ONE_DEG + (
            (PULSES_AT_THREE_DEG - PULSES_AT_ONE_DEG)
            * (magnitude - 1.0)
            / 2.0
        )
    return int(round(pulses if angle_deg > 0.0 else -pulses))


class BallEstimator:
    def __init__(self):
        self.valid = False
        self.position_mm = 0.0
        self.velocity_mm_s = 0.0
        self.measurement_time = 0.0
        self.confidence = 0.0
        self.outlier_candidate_mm = None
        self.outlier_candidate_count = 0
        self.rejected_measurement_count = 0

    def update(self, pixel_x, confidence, now):
        measured_mm = pixel_to_mm(pixel_x)
        if self.valid:
            dt = now - self.measurement_time
            maximum_jump_mm = max(
                OUTLIER_MIN_JUMP_MM,
                OUTLIER_MAX_SPEED_MM_S * max(0.0, dt),
            )
            if (
                0.003 <= dt <= 0.200
                and abs(measured_mm - self.position_mm)
                > maximum_jump_mm
            ):
                self.rejected_measurement_count += 1
                if (
                    self.outlier_candidate_mm is not None
                    and abs(measured_mm - self.outlier_candidate_mm)
                    <= OUTLIER_REACQUIRE_TOLERANCE_MM
                ):
                    self.outlier_candidate_count += 1
                else:
                    self.outlier_candidate_mm = measured_mm
                    self.outlier_candidate_count = 1

                if (
                    self.outlier_candidate_count
                    < OUTLIER_REACQUIRE_FRAMES
                ):
                    return False

                self.position_mm = measured_mm
                self.velocity_mm_s = 0.0
                self.measurement_time = now
                self.confidence = float(confidence)
                self.outlier_candidate_mm = None
                self.outlier_candidate_count = 0
                return True

            self.outlier_candidate_mm = None
            self.outlier_candidate_count = 0
            if 0.003 <= dt <= 0.200:
                measured_velocity = _clamp(
                    (measured_mm - self.position_mm) / dt,
                    -MAX_ABS_VELOCITY_MM_S,
                    MAX_ABS_VELOCITY_MM_S,
                )
                self.velocity_mm_s = (
                    (1.0 - VELOCITY_FILTER_ALPHA) * self.velocity_mm_s
                    + VELOCITY_FILTER_ALPHA * measured_velocity
                )
            else:
                self.velocity_mm_s = 0.0
        else:
            self.velocity_mm_s = 0.0
            self.valid = True

        self.position_mm = measured_mm
        self.measurement_time = now
        self.confidence = float(confidence)
        return True

    def estimate(self, now):
        if not self.valid:
            return 0.0, 0.0, 1e9, False
        age = max(0.0, now - self.measurement_time)
        position = self.position_mm + self.velocity_mm_s * min(
            age,
            MAX_PREDICTION_S,
        )
        return position, self.velocity_mm_s, age, age <= VISION_TIMEOUT_S


class BallTaskController:
    """Wait for a manually centered ball, then hold the 0 mm target."""

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
        self.target_mm = 0.0
        self.target_goal_mm = 0.0
        self.integral_error_mm_s = 0.0
        self.requested_beam_angle_deg = 0.0
        self.motor_target_pulses = 0
        self.stiction_active = False
        self.stiction_stationary_time_s = 0.0
        self.stiction_event_count = 0
        self.active_drive_limit_pulses = beam_angle_to_motor_pulses(
            MAX_BEAM_ANGLE_DEG
        )

    def force_fault(self, reason):
        self.state = "FAULT"
        self.fault_reason = str(reason)
        self.integral_error_mm_s = 0.0
        self.requested_beam_angle_deg = 0.0
        self.motor_target_pulses = 0
        self.stiction_active = False
        self.stiction_stationary_time_s = 0.0
        self.active_drive_limit_pulses = 0

    def _update_state(self, position_mm, velocity_mm_s, valid, now):
        if self.state == "FAULT":
            return
        if self.state == "PIXEL_CALIBRATION":
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
            else:
                self.arm_since = None
            return
        if not valid:
            self.force_fault("vision timeout")
        elif abs(position_mm) >= BALL_EDGE_LIMIT_MM:
            self.force_fault("ball edge")
        elif (
            self.state != "HOLD_NEGATIVE"
            and now - self.start_time >= STEP_TEST_TIMEOUT_S
        ):
            self.force_fault("step timeout")

        if self.state == "HOLD_CENTER":
            self.target_goal_mm = 0.0
            if now - self.state_since >= CENTER_HOLD_BEFORE_STEP_S:
                self.state = "TO_POSITIVE"
                self.state_since = now
                self.target_goal_mm = POSITIVE_TARGET_MM
                self.integral_error_mm_s = 0.0
            return

        if self.state == "TO_POSITIVE":
            self.target_goal_mm = POSITIVE_TARGET_MM
            if (
                abs(position_mm - POSITIVE_TARGET_MM)
                <= POSITION_SETTLE_ERROR_MM
                and abs(velocity_mm_s) <= POSITION_SETTLE_SPEED_MM_S
                and abs(self.target_mm - POSITIVE_TARGET_MM) <= 0.5
            ):
                if self.settle_since is None:
                    self.settle_since = now
                elif now - self.settle_since >= POSITION_SETTLE_TIME_S:
                    self.state = "HOLD_POSITIVE"
                    self.state_since = now
                    self.settle_since = None
            else:
                self.settle_since = None
            return

        if self.state == "HOLD_POSITIVE":
            self.target_goal_mm = POSITIVE_TARGET_MM
            if (
                not POSITIVE_ONLY_TEST
                and now - self.state_since >= POSITIVE_HOLD_TIME_S
            ):
                self.state = "TO_NEGATIVE"
                self.state_since = now
                self.target_goal_mm = NEGATIVE_TARGET_MM
                self.integral_error_mm_s = 0.0
            return

        if self.state == "TO_NEGATIVE":
            self.target_goal_mm = NEGATIVE_TARGET_MM
            if (
                abs(position_mm - NEGATIVE_TARGET_MM)
                <= POSITION_SETTLE_ERROR_MM
                and abs(velocity_mm_s) <= POSITION_SETTLE_SPEED_MM_S
                and abs(self.target_mm - NEGATIVE_TARGET_MM) <= 0.5
            ):
                if self.settle_since is None:
                    self.settle_since = now
                elif now - self.settle_since >= POSITION_SETTLE_TIME_S:
                    self.state = "HOLD_NEGATIVE"
                    self.state_since = now
                    self.settle_since = None
            else:
                self.settle_since = None
            return

        if self.state == "HOLD_NEGATIVE":
            self.target_goal_mm = NEGATIVE_TARGET_MM

    def _slew_target(self, dt):
        difference = self.target_goal_mm - self.target_mm
        slew_mm_s = (
            NEGATIVE_REFERENCE_SLEW_MM_S
            if difference < 0.0
            else REFERENCE_SLEW_MM_S
        )
        maximum_step = slew_mm_s * dt
        self.target_mm += _clamp(
            difference,
            -maximum_step,
            maximum_step,
        )

    def _apply_stiction_compensation(
        self,
        requested_angle_deg,
        error_mm,
        velocity_mm_s,
        dt,
    ):
        eligible = (
            self.state == "TO_POSITIVE"
            and error_mm >= POSITIVE_STICTION_TRIGGER_ERROR_MM
            and requested_angle_deg > 0.0
        )

        if self.stiction_active:
            if (
                not eligible
                or velocity_mm_s >= POSITIVE_STICTION_RELEASE_SPEED_MM_S
            ):
                self.stiction_active = False
                self.stiction_stationary_time_s = 0.0
        elif (
            eligible
            and abs(velocity_mm_s)
            <= POSITIVE_STICTION_TRIGGER_SPEED_MM_S
        ):
            self.stiction_stationary_time_s += dt
            if (
                self.stiction_stationary_time_s
                >= POSITIVE_STICTION_TRIGGER_TIME_S
            ):
                self.stiction_active = True
                self.stiction_event_count += 1
        else:
            self.stiction_stationary_time_s = 0.0

        if self.stiction_active:
            return max(
                requested_angle_deg,
                POSITIVE_STICTION_MIN_ANGLE_DEG,
            )
        return requested_angle_deg

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
        self._slew_target(dt)

        if self.state not in (
            "WAIT_CENTER",
            "PIXEL_CALIBRATION",
            "FAULT",
        ) and valid:
            error_mm = self.target_mm - position_mm
            integral_enabled = self.state in (
                "HOLD_CENTER",
                "HOLD_POSITIVE",
                "HOLD_NEGATIVE",
            )
            if (
                integral_enabled
                and abs(error_mm) <= INTEGRAL_ACTIVE_ERROR_MM
                and abs(velocity_mm_s) <= INTEGRAL_MAX_SPEED_MM_S
            ):
                self.integral_error_mm_s += error_mm * dt
                integral_limit = (
                    INTEGRAL_ANGLE_LIMIT_DEG
                    / POSITION_KI_DEG_PER_MM_S
                )
                self.integral_error_mm_s = _clamp(
                    self.integral_error_mm_s,
                    -integral_limit,
                    integral_limit,
                )
            else:
                self.integral_error_mm_s *= max(
                    0.0,
                    1.0 - INTEGRAL_DECAY_PER_S * dt,
                )
            integral_angle_deg = (
                POSITION_KI_DEG_PER_MM_S
                * self.integral_error_mm_s
            )
            requested_angle = (
                POSITION_KP_DEG_PER_MM * error_mm
                + integral_angle_deg
                - VELOCITY_KD_DEG_PER_MM_S * velocity_mm_s
            )
            requested_angle = self._apply_stiction_compensation(
                requested_angle,
                error_mm,
                velocity_mm_s,
                dt,
            )
            self.requested_beam_angle_deg = _clamp(
                requested_angle,
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
            self.integral_error_mm_s = 0.0
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
            "requested_beam_angle_deg": self.requested_beam_angle_deg,
            "integral_angle_deg": (
                POSITION_KI_DEG_PER_MM_S * self.integral_error_mm_s
            ),
            "stiction_active": self.stiction_active,
            "stiction_stationary_ms": (
                self.stiction_stationary_time_s * 1000.0
            ),
            "stiction_event_count": self.stiction_event_count,
            "motor_target_pulses": self.motor_target_pulses,
            "drive_limit_pulses": self.active_drive_limit_pulses,
            "elapsed_s": elapsed_s,
        }
