"""Pure-Python position calibration and static ball-task controller."""


CONTROL_PROFILE_NAME = "linkage_v1"

POSITIVE_50_PX = 212.0
CENTER_0_PX = 324.0
NEGATIVE_50_PX = 435.0

POSITIVE_TARGET_MM = 50.0
NEGATIVE_TARGET_MM = -50.0
POSITIVE_TARGET_SLEW_MM_S = 90.0
NEGATIVE_TARGET_SLEW_MM_S = 70.0

ARM_CENTER_TOLERANCE_MM = 12.0
ARM_MAX_SPEED_MM_S = 35.0
ARM_STABLE_TIME_S = 0.50
VISION_TIMEOUT_S = 0.150
MAX_PREDICTION_S = 0.050
MAX_ABS_VELOCITY_MM_S = 1500.0
VELOCITY_FILTER_ALPHA = 0.45

KP_PULSES_PER_MM = 3.00
KI_PULSES_PER_MM_S = 0.08
KV_POSITIVE_PULSES_PER_MM_S = 2.50
KV_NEGATIVE_PULSES_PER_MM_S = 3.00
KV_HOLD_PULSES_PER_MM_S = 2.20
NEAR_TARGET_VELOCITY_GAIN_SCALE = 1.20
INTEGRAL_ACTIVE_ERROR_MM = 35.0
INTEGRAL_GOAL_WINDOW_MM = 25.0
INTEGRAL_MAX_SPEED_MM_S = 60.0
INTEGRAL_LIMIT_MM_S = 200.0
MOTOR_TARGET_LIMIT_PULSES = 120.0
POSITIVE_DRIVE_LIMIT_PULSES = 75.0
NEGATIVE_DRIVE_LIMIT_PULSES = 100.0

LINKAGE_FINE_DISTANCE_MM = 8.0
LINKAGE_NEAR_DISTANCE_MM = 25.0
LINKAGE_FULL_DISTANCE_MM = 50.0
LINKAGE_FINE_DRIVE_LIMIT_PULSES = 40.0
LINKAGE_NEAR_DRIVE_LIMIT_PULSES = 65.0
HOLD_MOTOR_LIMIT_PULSES = 45.0

POSITIVE_REACHED_MM = 45.0
POSITIVE_SETTLE_SPEED_MM_S = 35.0
POSITIVE_SETTLE_TIME_S = 0.20
NEGATIVE_SETTLE_ERROR_MM = 5.0
NEGATIVE_SETTLE_SPEED_MM_S = 20.0
NEGATIVE_SETTLE_TIME_S = 0.40
TASK_TIMEOUT_S = 7.0
WRONG_DIRECTION_LIMIT_MM = 15.0
WRONG_DIRECTION_GRACE_S = 0.35
BALL_EDGE_LIMIT_MM = 105.0


def _clamp(value, low, high):
    return max(low, min(high, value))


def pixel_to_mm(pixel_x):
    """Piecewise three-point calibration; image-left is positive."""

    pixel_x = float(pixel_x)
    if pixel_x <= CENTER_0_PX:
        return (
            (CENTER_0_PX - pixel_x)
            * POSITIVE_TARGET_MM
            / (CENTER_0_PX - POSITIVE_50_PX)
        )
    return (
        -(pixel_x - CENTER_0_PX)
        * abs(NEGATIVE_TARGET_MM)
        / (NEGATIVE_50_PX - CENTER_0_PX)
    )


class BallEstimator:
    def __init__(self):
        self.valid = False
        self.position_mm = 0.0
        self.velocity_mm_s = 0.0
        self.measurement_time = 0.0
        self.confidence = 0.0

    def update(self, pixel_x, confidence, now):
        measured_mm = pixel_to_mm(pixel_x)
        if self.valid:
            dt = now - self.measurement_time
            if 0.003 <= dt <= 0.200:
                measured_velocity = (
                    measured_mm - self.position_mm
                ) / dt
                measured_velocity = _clamp(
                    measured_velocity,
                    -MAX_ABS_VELOCITY_MM_S,
                    MAX_ABS_VELOCITY_MM_S,
                )
                keep = 1.0 - VELOCITY_FILTER_ALPHA
                self.velocity_mm_s = (
                    keep * self.velocity_mm_s
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

    def estimate(self, now):
        if not self.valid:
            return 0.0, 0.0, 1e9, False
        age = max(0.0, now - self.measurement_time)
        predicted_position = (
            self.position_mm
            + self.velocity_mm_s * min(age, MAX_PREDICTION_S)
        )
        return (
            predicted_position,
            self.velocity_mm_s,
            age,
            age <= VISION_TIMEOUT_S,
        )


class BallTaskController:
    """Automatic O -> +50 mm -> -50 mm bench-task state machine."""

    def __init__(self):
        self.estimator = BallEstimator()
        self.state = "WAIT_CENTER"
        self.fault_reason = ""
        self.target_goal_mm = 0.0
        self.target_mm = 0.0
        self.integral_mm_s = 0.0
        self.motor_target_pulses = 0
        self.active_drive_limit_pulses = 0
        self.arm_since = None
        self.positive_stable_since = None
        self.negative_stable_since = None
        self.start_time = None
        self.positive_reached_time = None
        self.finish_time = None
        self.last_update_time = None

    def force_fault(self, reason):
        self.state = "FAULT"
        self.fault_reason = str(reason)
        self.target_goal_mm = 0.0
        self.target_mm = 0.0
        self.integral_mm_s = 0.0
        self.motor_target_pulses = 0
        self.active_drive_limit_pulses = 0

    def _slew_target(self, dt):
        if (
            self.state in ("TO_NEGATIVE", "TIMEOUT")
            and self.target_goal_mm < self.target_mm
        ):
            slew_mm_s = NEGATIVE_TARGET_SLEW_MM_S
        else:
            slew_mm_s = POSITIVE_TARGET_SLEW_MM_S
        maximum_step = slew_mm_s * dt
        difference = self.target_goal_mm - self.target_mm
        self.target_mm += _clamp(
            difference,
            -maximum_step,
            maximum_step,
        )

    def _update_state(self, position_mm, velocity_mm_s, valid, now):
        if self.state == "FAULT":
            return

        if self.state == "WAIT_CENTER":
            self.target_goal_mm = 0.0
            self.target_mm = 0.0
            if (
                valid
                and abs(position_mm) <= ARM_CENTER_TOLERANCE_MM
                and abs(velocity_mm_s) <= ARM_MAX_SPEED_MM_S
            ):
                if self.arm_since is None:
                    self.arm_since = now
                elif now - self.arm_since >= ARM_STABLE_TIME_S:
                    self.state = "TO_POSITIVE"
                    self.start_time = now
                    self.target_goal_mm = POSITIVE_TARGET_MM
                    self.integral_mm_s = 0.0
            else:
                self.arm_since = None
            return

        if not valid:
            self.force_fault("vision timeout")
            return

        elapsed = now - self.start_time
        if abs(position_mm) >= BALL_EDGE_LIMIT_MM:
            self.force_fault("ball edge")
            return

        if (
            self.state == "TO_POSITIVE"
            and elapsed >= WRONG_DIRECTION_GRACE_S
            and position_mm <= -WRONG_DIRECTION_LIMIT_MM
        ):
            self.force_fault("direction mismatch")
            return

        if (
            self.state == "TO_NEGATIVE"
            and self.positive_reached_time is not None
            and now - self.positive_reached_time >= TASK_TIMEOUT_S
        ):
            self.state = "TIMEOUT"
            self.target_goal_mm = NEGATIVE_TARGET_MM

        if self.state == "TO_POSITIVE":
            self.target_goal_mm = POSITIVE_TARGET_MM
            if (
                self.target_mm >= POSITIVE_REACHED_MM
                and position_mm >= POSITIVE_REACHED_MM
                and abs(velocity_mm_s) <= POSITIVE_SETTLE_SPEED_MM_S
            ):
                if self.positive_stable_since is None:
                    self.positive_stable_since = now
                elif (
                    now - self.positive_stable_since
                    >= POSITIVE_SETTLE_TIME_S
                ):
                    self.state = "TO_NEGATIVE"
                    self.positive_reached_time = now
                    self.target_goal_mm = NEGATIVE_TARGET_MM
                    self.integral_mm_s = 0.0
                    self.positive_stable_since = None
            else:
                self.positive_stable_since = None
            return

        if self.state in ("TO_NEGATIVE", "TIMEOUT"):
            self.target_goal_mm = NEGATIVE_TARGET_MM
            if (
                abs(position_mm - NEGATIVE_TARGET_MM)
                <= NEGATIVE_SETTLE_ERROR_MM
                and abs(velocity_mm_s) <= NEGATIVE_SETTLE_SPEED_MM_S
            ):
                if self.negative_stable_since is None:
                    self.negative_stable_since = now
                elif (
                    self.state == "TO_NEGATIVE"
                    and now - self.negative_stable_since
                    >= NEGATIVE_SETTLE_TIME_S
                ):
                    self.state = "HOLD_NEGATIVE"
                    self.finish_time = now
                    self.target_mm = NEGATIVE_TARGET_MM
                    self.integral_mm_s = 0.0
            else:
                self.negative_stable_since = None
            return

        if self.state == "HOLD_NEGATIVE":
            self.target_goal_mm = NEGATIVE_TARGET_MM

    def _scheduled_drive_limit(self, distance_mm, full_limit):
        """Taper target-directed drive as the ball approaches its goal."""

        distance_mm = abs(float(distance_mm))
        if distance_mm <= LINKAGE_FINE_DISTANCE_MM:
            return LINKAGE_FINE_DRIVE_LIMIT_PULSES
        if distance_mm < LINKAGE_NEAR_DISTANCE_MM:
            blend = (
                (distance_mm - LINKAGE_FINE_DISTANCE_MM)
                / (LINKAGE_NEAR_DISTANCE_MM - LINKAGE_FINE_DISTANCE_MM)
            )
            return (
                LINKAGE_FINE_DRIVE_LIMIT_PULSES
                + blend
                * (
                    LINKAGE_NEAR_DRIVE_LIMIT_PULSES
                    - LINKAGE_FINE_DRIVE_LIMIT_PULSES
                )
            )
        if distance_mm < LINKAGE_FULL_DISTANCE_MM:
            blend = (
                (distance_mm - LINKAGE_NEAR_DISTANCE_MM)
                / (LINKAGE_FULL_DISTANCE_MM - LINKAGE_NEAR_DISTANCE_MM)
            )
            return (
                LINKAGE_NEAR_DRIVE_LIMIT_PULSES
                + blend
                * (full_limit - LINKAGE_NEAR_DRIVE_LIMIT_PULSES)
            )
        return float(full_limit)

    def _calculate_motor_target(
        self,
        position_mm,
        velocity_mm_s,
        valid,
        dt,
    ):
        if self.state in ("WAIT_CENTER", "FAULT") or not valid:
            self.integral_mm_s = 0.0
            self.active_drive_limit_pulses = 0
            return 0

        error_mm = self.target_mm - position_mm
        goal_error_mm = self.target_goal_mm - position_mm
        if (
            abs(error_mm) <= INTEGRAL_ACTIVE_ERROR_MM
            and abs(goal_error_mm) <= INTEGRAL_GOAL_WINDOW_MM
            and abs(velocity_mm_s) <= INTEGRAL_MAX_SPEED_MM_S
        ):
            self.integral_mm_s += error_mm * dt
            self.integral_mm_s = _clamp(
                self.integral_mm_s,
                -INTEGRAL_LIMIT_MM_S,
                INTEGRAL_LIMIT_MM_S,
            )
        else:
            self.integral_mm_s *= 0.98

        if self.state in ("TO_NEGATIVE", "TIMEOUT"):
            velocity_gain = KV_NEGATIVE_PULSES_PER_MM_S
        elif self.state == "HOLD_NEGATIVE":
            velocity_gain = KV_HOLD_PULSES_PER_MM_S
        else:
            velocity_gain = KV_POSITIVE_PULSES_PER_MM_S

        if (
            self.state in ("TO_POSITIVE", "TO_NEGATIVE", "TIMEOUT")
            and abs(goal_error_mm) <= LINKAGE_NEAR_DISTANCE_MM
        ):
            velocity_gain *= NEAR_TARGET_VELOCITY_GAIN_SCALE

        positive_ball_acceleration = (
            KP_PULSES_PER_MM * error_mm
            + KI_PULSES_PER_MM_S * self.integral_mm_s
            - velocity_gain * velocity_mm_s
        )

        # Hardware test on 2026-07-30 established the actual installed sign:
        # signed positive / Emm CCW accelerates the ball toward image-left
        # (+x), while signed negative / Emm CW accelerates it toward -x.
        motor_target = positive_ball_acceleration
        if self.state == "HOLD_NEGATIVE":
            self.active_drive_limit_pulses = int(
                HOLD_MOTOR_LIMIT_PULSES
            )
            minimum_target = -HOLD_MOTOR_LIMIT_PULSES
            maximum_target = HOLD_MOTOR_LIMIT_PULSES
        else:
            full_drive_limit = (
                NEGATIVE_DRIVE_LIMIT_PULSES
                if self.state in ("TO_NEGATIVE", "TIMEOUT")
                else POSITIVE_DRIVE_LIMIT_PULSES
            )
            drive_limit = self._scheduled_drive_limit(
                abs(goal_error_mm),
                full_drive_limit,
            )
            self.active_drive_limit_pulses = int(round(drive_limit))
            minimum_target = -MOTOR_TARGET_LIMIT_PULSES
            maximum_target = MOTOR_TARGET_LIMIT_PULSES

            # Taper only acceleration toward the goal. Opposite-signed
            # braking remains available up to the full motor limit.
            if goal_error_mm < 0.0:
                minimum_target = -drive_limit
            elif goal_error_mm > 0.0:
                maximum_target = drive_limit

        return int(
            round(
                _clamp(
                    motor_target,
                    minimum_target,
                    maximum_target,
                )
            )
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
        self._slew_target(dt)
        self.motor_target_pulses = self._calculate_motor_target(
            position_mm,
            velocity_mm_s,
            valid,
            dt,
        )

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
            "motor_target_pulses": self.motor_target_pulses,
            "drive_limit_pulses": self.active_drive_limit_pulses,
            "elapsed_s": elapsed_s,
        }
