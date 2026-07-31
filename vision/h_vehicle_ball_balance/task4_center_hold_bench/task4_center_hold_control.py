"""Task 4 bench controller: reject disturbances and hold the ball at 0 mm."""


CONTROL_PROFILE_NAME = "task4_ramped_breakaway_v6"

# Current 2026-07-31 three-point camera calibration.
# Image-left is the positive beam direction.
POSITIVE_50_PX = 212.0
CENTER_0_PX = 324.0
NEGATIVE_50_PX = 435.0

POSITIVE_50_MM = 50.0
NEGATIVE_50_MM = -50.0

# Vision estimator and safety limits.
VISION_TIMEOUT_S = 0.150
MAX_PREDICTION_S = 0.050
MAX_ABS_VELOCITY_MM_S = 1000.0
VELOCITY_FILTER_ALPHA = 0.35
OUTLIER_MIN_JUMP_MM = 12.0
OUTLIER_MAX_SPEED_MM_S = 300.0
OUTLIER_REACQUIRE_FRAMES = 2
OUTLIER_REACQUIRE_TOLERANCE_MM = 8.0
# Automatic acquisition from any detected ball position. The outlier filter
# already requires a persistent detection after a large jump, so only a short
# confirmation window is needed here.
ACQUIRE_STABLE_TIME_S = 0.05

# Task 4 acceptance and recovery-state thresholds.
SPEC_ERROR_LIMIT_MM = 10.0
RECOVERY_ENTER_ERROR_MM = 8.0
RECOVERY_ENTER_SPEED_MM_S = 30.0
RECOVERY_EXIT_ERROR_MM = 5.0
RECOVERY_EXIT_SPEED_MM_S = 12.0
RECOVERY_SETTLE_TIME_S = 0.40

# Position outer loop. The far profile provides stronger recovery after a
# disturbance; the near profile avoids exciting oscillation around O.
NEAR_POSITION_KP_DEG_PER_MM = 0.060
NEAR_VELOCITY_KD_DEG_PER_MM_S = 0.028
FAR_POSITION_KP_DEG_PER_MM = 0.045
FAR_VELOCITY_KD_DEG_PER_MM_S = 0.024
GAIN_SCHEDULE_ERROR_MM = 8.0

# Integral is deliberately restricted to the quiet center region. It removes
# beam-zero bias without winding up during a large disturbance.
POSITION_KI_DEG_PER_MM_S = 0.010
INTEGRAL_ACTIVE_ERROR_MM = 12.0
INTEGRAL_MAX_SPEED_MM_S = 12.0
INTEGRAL_ANGLE_LIMIT_DEG = 0.50
INTEGRAL_DECAY_PER_S = 2.0

# Beam-angle and linkage limits. The linkage was measured as:
# 0 deg -> 0 pulse, 1 deg -> 80 pulse, 3 deg -> 160 pulse.
HOLD_MAX_BEAM_ANGLE_DEG = 3.20
RECOVERY_MAX_BEAM_ANGLE_DEG = 3.90
CALIBRATED_MAX_BEAM_ANGLE_DEG = 4.00
ANGLE_DEADBAND_DEG = 0.025
PULSES_AT_ONE_DEG = 80.0
PULSES_AT_THREE_DEG = 160.0

# Explicit low-speed breakaway assistance. It is only active while the ball is
# measurably away from O and stationary, so it cannot continuously kick at O.
STICTION_TRIGGER_ERROR_MM = 4.0
STICTION_TRIGGER_SPEED_MM_S = 8.0
STICTION_TRIGGER_TIME_S = 0.10
STICTION_RELEASE_SPEED_MM_S = 12.0
POSITIVE_STICTION_MIN_ANGLE_DEG = 2.20
NEGATIVE_STICTION_MIN_ANGLE_DEG = 2.00
POSITIVE_STICTION_MAX_ANGLE_DEG = 3.20
NEGATIVE_STICTION_MAX_ANGLE_DEG = 3.00
STICTION_RAMP_DEG_PER_S = 2.50


def _clamp(value, low, high):
    return max(low, min(high, value))


def pixel_to_mm(pixel_x):
    """Convert the ball-center pixel to signed millimetres from O."""

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
    elif magnitude <= 3.0:
        pulses = PULSES_AT_ONE_DEG + (
            (PULSES_AT_THREE_DEG - PULSES_AT_ONE_DEG)
            * (magnitude - 1.0)
            / 2.0
        )
    else:
        # No-load linkage clearance was confirmed by the user beyond the
        # original 3 degree calibration. Continue the measured 1..3 degree
        # terminal slope (40 pulse/degree) up to the 4 degree software limit.
        pulses = PULSES_AT_THREE_DEG + 40.0 * (magnitude - 3.0)
    return int(round(pulses if angle_deg > 0.0 else -pulses))


class BallEstimator:
    """Position/velocity estimator with single-frame false-detection rejection."""

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
                and abs(measured_mm - self.position_mm) > maximum_jump_mm
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

                if self.outlier_candidate_count < OUTLIER_REACQUIRE_FRAMES:
                    return False

                # A persistent new position is a real disturbance, not a false
                # detection. Reacquire it with zero initial velocity.
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
        position_mm = self.position_mm + self.velocity_mm_s * min(
            age,
            MAX_PREDICTION_S,
        )
        return (
            position_mm,
            self.velocity_mm_s,
            age,
            age <= VISION_TIMEOUT_S,
        )


class BallTaskController:
    """Arm at O, then hold O indefinitely and record disturbance recovery."""

    def __init__(self, pixel_calibration_only=False):
        self.estimator = BallEstimator()
        self.pixel_calibration_only = bool(pixel_calibration_only)
        self.state = (
            "PIXEL_CALIBRATION"
            if self.pixel_calibration_only
            else "SEARCH_BALL"
        )
        self.fault_reason = ""
        self.arm_since = None
        self.start_time = None
        self.last_update_time = None
        self.recovery_since = None
        self.recovery_settle_since = None
        self.last_recovery_time_s = 0.0
        self.recovery_count = 0
        self.peak_abs_error_mm = 0.0
        self.out_of_spec = False
        self.integral_error_mm_s = 0.0
        self.requested_beam_angle_deg = 0.0
        self.motor_target_pulses = 0
        self.active_drive_limit_pulses = 0
        self.stiction_active = False
        self.stiction_stationary_time_s = 0.0
        self.stiction_event_count = 0

    def force_fault(self, reason):
        """Compatibility entry for non-vision hardware failures.

        Vision loss never calls this method. A motor/UART failure must inhibit
        commands, but the user-facing state is SAFE_STOP rather than FAULT.
        """

        self.state = "SAFE_STOP"
        self.fault_reason = str(reason)
        self.integral_error_mm_s = 0.0
        self.requested_beam_angle_deg = 0.0
        self.motor_target_pulses = 0
        self.active_drive_limit_pulses = 0
        self.stiction_active = False
        self.stiction_stationary_time_s = 0.0

    def _update_state(self, position_mm, velocity_mm_s, valid, now):
        if self.state in ("SAFE_STOP", "PIXEL_CALIBRATION"):
            return

        if self.state == "SEARCH_BALL":
            if valid:
                if self.arm_since is None:
                    self.arm_since = now
                elif now - self.arm_since >= ACQUIRE_STABLE_TIME_S:
                    self.start_time = now
                    self.fault_reason = ""
                    if (
                        abs(position_mm) >= RECOVERY_ENTER_ERROR_MM
                        or abs(velocity_mm_s)
                        >= RECOVERY_ENTER_SPEED_MM_S
                    ):
                        self.state = "RECOVER_CENTER"
                        self.recovery_since = now
                        self.recovery_count = 1
                    else:
                        self.state = "HOLD_CENTER"
            else:
                self.arm_since = None
            return

        if not valid:
            self.state = "SEARCH_BALL"
            self.arm_since = None
            self.recovery_since = None
            self.recovery_settle_since = None
            self.integral_error_mm_s = 0.0
            self.fault_reason = ""
            return

        abs_error_mm = abs(position_mm)
        self.peak_abs_error_mm = max(self.peak_abs_error_mm, abs_error_mm)
        self.out_of_spec = abs_error_mm > SPEC_ERROR_LIMIT_MM

        if self.state == "HOLD_CENTER":
            if (
                abs_error_mm >= RECOVERY_ENTER_ERROR_MM
                or abs(velocity_mm_s) >= RECOVERY_ENTER_SPEED_MM_S
            ):
                self.state = "RECOVER_CENTER"
                self.recovery_since = now
                self.recovery_settle_since = None
                self.recovery_count += 1
                self.integral_error_mm_s = 0.0
            return

        if self.state == "RECOVER_CENTER":
            if (
                abs_error_mm <= RECOVERY_EXIT_ERROR_MM
                and abs(velocity_mm_s) <= RECOVERY_EXIT_SPEED_MM_S
            ):
                if self.recovery_settle_since is None:
                    self.recovery_settle_since = now
                elif (
                    now - self.recovery_settle_since
                    >= RECOVERY_SETTLE_TIME_S
                ):
                    self.state = "HOLD_CENTER"
                    self.last_recovery_time_s = max(
                        0.0,
                        now - self.recovery_since,
                    )
                    self.recovery_since = None
                    self.recovery_settle_since = None
            else:
                self.recovery_settle_since = None

    def _update_integral(self, error_mm, velocity_mm_s, dt):
        if (
            abs(error_mm) <= INTEGRAL_ACTIVE_ERROR_MM
            and abs(velocity_mm_s) <= INTEGRAL_MAX_SPEED_MM_S
        ):
            self.integral_error_mm_s += error_mm * dt
            integral_limit = (
                INTEGRAL_ANGLE_LIMIT_DEG / POSITION_KI_DEG_PER_MM_S
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

    def _apply_stiction_compensation(
        self,
        requested_angle_deg,
        error_mm,
        velocity_mm_s,
        dt,
    ):
        eligible = (
            abs(error_mm) >= STICTION_TRIGGER_ERROR_MM
            and abs(velocity_mm_s) <= STICTION_TRIGGER_SPEED_MM_S
            and requested_angle_deg * error_mm > 0.0
        )

        if self.stiction_active:
            if (
                not eligible
                or abs(velocity_mm_s) >= STICTION_RELEASE_SPEED_MM_S
            ):
                self.stiction_active = False
                self.stiction_stationary_time_s = 0.0
            else:
                self.stiction_stationary_time_s += dt
        elif eligible:
            self.stiction_stationary_time_s += dt
            if (
                self.stiction_stationary_time_s
                >= STICTION_TRIGGER_TIME_S
            ):
                self.stiction_active = True
                self.stiction_event_count += 1
        else:
            self.stiction_stationary_time_s = 0.0

        if not self.stiction_active:
            return requested_angle_deg
        ramp_time_s = max(
            0.0,
            self.stiction_stationary_time_s - STICTION_TRIGGER_TIME_S,
        )
        if requested_angle_deg > 0.0:
            breakaway_angle_deg = min(
                POSITIVE_STICTION_MAX_ANGLE_DEG,
                POSITIVE_STICTION_MIN_ANGLE_DEG
                + STICTION_RAMP_DEG_PER_S * ramp_time_s,
            )
            return max(
                requested_angle_deg,
                breakaway_angle_deg,
            )
        breakaway_angle_deg = min(
            NEGATIVE_STICTION_MAX_ANGLE_DEG,
            NEGATIVE_STICTION_MIN_ANGLE_DEG
            + STICTION_RAMP_DEG_PER_S * ramp_time_s,
        )
        return min(
            requested_angle_deg,
            -breakaway_angle_deg,
        )

    def _calculate_control(self, position_mm, velocity_mm_s, valid, dt):
        if self.state not in ("HOLD_CENTER", "RECOVER_CENTER") or not valid:
            self.integral_error_mm_s = 0.0
            self.requested_beam_angle_deg = 0.0
            self.motor_target_pulses = 0
            self.active_drive_limit_pulses = 0
            self.stiction_active = False
            self.stiction_stationary_time_s = 0.0
            return

        error_mm = -position_mm
        self._update_integral(error_mm, velocity_mm_s, dt)

        if abs(error_mm) >= GAIN_SCHEDULE_ERROR_MM:
            kp = FAR_POSITION_KP_DEG_PER_MM
            kd = FAR_VELOCITY_KD_DEG_PER_MM_S
        else:
            kp = NEAR_POSITION_KP_DEG_PER_MM
            kd = NEAR_VELOCITY_KD_DEG_PER_MM_S

        integral_angle_deg = (
            POSITION_KI_DEG_PER_MM_S * self.integral_error_mm_s
        )
        requested_angle_deg = (
            kp * error_mm
            + integral_angle_deg
            - kd * velocity_mm_s
        )
        requested_angle_deg = self._apply_stiction_compensation(
            requested_angle_deg,
            error_mm,
            velocity_mm_s,
            dt,
        )

        angle_limit_deg = (
            RECOVERY_MAX_BEAM_ANGLE_DEG
            if self.state == "RECOVER_CENTER"
            else HOLD_MAX_BEAM_ANGLE_DEG
        )
        self.requested_beam_angle_deg = _clamp(
            requested_angle_deg,
            -angle_limit_deg,
            angle_limit_deg,
        )
        self.motor_target_pulses = beam_angle_to_motor_pulses(
            self.requested_beam_angle_deg
        )
        self.active_drive_limit_pulses = beam_angle_to_motor_pulses(
            angle_limit_deg
        )

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
        self._calculate_control(position_mm, velocity_mm_s, valid, dt)

        elapsed_s = (
            0.0
            if self.start_time is None
            else max(0.0, now - self.start_time)
        )
        current_recovery_time_s = (
            0.0
            if self.recovery_since is None
            else max(0.0, now - self.recovery_since)
        )
        return {
            "state": self.state,
            "fault_reason": self.fault_reason,
            "position_mm": position_mm,
            "velocity_mm_s": velocity_mm_s,
            "age_ms": age_s * 1000.0,
            "valid": valid,
            "confidence": self.estimator.confidence,
            "target_mm": 0.0,
            "target_goal_mm": 0.0,
            "reference_velocity_mm_s": 0.0,
            "reference_acceleration_mm_s2": 0.0,
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
            "vision_reject_count": (
                self.estimator.rejected_measurement_count
            ),
            "elapsed_s": elapsed_s,
            "out_of_spec": self.out_of_spec,
            "peak_abs_error_mm": self.peak_abs_error_mm,
            "recovery_count": self.recovery_count,
            "current_recovery_time_s": current_recovery_time_s,
            "last_recovery_time_s": self.last_recovery_time_s,
        }
