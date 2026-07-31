"""Fixed-phase full task controller with vision-gated transitions."""

from center_control import (
    BallEstimator,
    CENTER_0_PX,
    NEGATIVE_50_PX,
    POSITIVE_50_PX,
    beam_angle_to_motor_pulses,
)


CONTROL_PROFILE_NAME = "timed_full_task_v3_3_3"

ARM_CENTER_TOLERANCE_MM = 8.0
ARM_MAX_SPEED_MM_S = 20.0
ARM_STABLE_TIME_S = 0.35
CENTER_HOLD_TIME_S = 0.25
BALL_EDGE_LIMIT_MM = 105.0
TASK_TIME_LIMIT_S = 5.0

POSITIVE_TARGET_MM = 50.0
NEGATIVE_TARGET_MM = -50.0

# Fixed action table. These are intentionally centralized for bench tuning.
POSITIVE_PUSH_ANGLE_DEG = 2.20
POSITIVE_PUSH_MAX_TIME_S = 1.20
POSITIVE_PUSH_MIN_TIME_S = 0.20
POSITIVE_BREAKAWAY_DELAY_S = 0.65
POSITIVE_BREAKAWAY_ANGLE_DEG = 3.00
POSITIVE_BREAKAWAY_POSITION_MM = 6.0
POSITIVE_BREAKAWAY_SPEED_MM_S = 8.0
POSITIVE_BRAKE_POSITION_MM = 5.0
POSITIVE_BRAKE_SPEED_MM_S = 25.0

POSITIVE_BRAKE_ANGLE_DEG = -2.30
POSITIVE_BRAKE_MAX_TIME_S = 0.28
POSITIVE_BRAKE_MIN_TIME_S = 0.20
POSITIVE_SETTLE_ENTRY_MM = 42.0
POSITIVE_SETTLE_ENTRY_SPEED_MM_S = 30.0

NEGATIVE_PUSH_ANGLE_DEG = -1.40
NEGATIVE_PUSH_MAX_TIME_S = 1.15
NEGATIVE_PUSH_MIN_TIME_S = 0.30
NEGATIVE_BRAKE_POSITION_MM = 16.0
NEGATIVE_BRAKE_SPEED_MM_S = -95.0
NEGATIVE_PREDICTIVE_BRAKE_POSITION_MM = 25.0
NEGATIVE_PREDICTIVE_BRAKE_SPEED_MM_S = -70.0

NEGATIVE_BRAKE_ANGLE_DEG = 1.20
NEGATIVE_BRAKE_MAX_TIME_S = 1.00
NEGATIVE_BRAKE_MIN_TIME_S = 0.25
NEGATIVE_SETTLE_ENTRY_MM = -42.0
NEGATIVE_SETTLE_ENTRY_SPEED_MM_S = 30.0
NEGATIVE_BRAKE_EARLY_EXIT_SPEED_MM_S = -40.0

SETTLE_POSITION_KP_DEG_PER_MM = 0.045
SETTLE_VELOCITY_KD_DEG_PER_MM_S = 0.018
SETTLE_MAX_ANGLE_DEG = 2.20
SETTLE_STICTION_MIN_ANGLE_DEG = 1.10
POSITIVE_SETTLE_STICTION_MIN_ANGLE_DEG = 2.20
SETTLE_STICTION_ERROR_MM = 5.0
SETTLE_STICTION_SPEED_MM_S = 6.0

POSITIVE_SETTLE_ERROR_MM = 10.0
POSITIVE_SETTLE_SPEED_MM_S = 20.0
POSITIVE_SETTLE_TIME_S = 0.12
POSITIVE_SETTLE_TIMEOUT_S = 1.40
FINAL_SETTLE_ERROR_MM = 9.0
FINAL_SETTLE_SPEED_MM_S = 12.0
FINAL_SETTLE_TIME_S = 0.0

MAX_BEAM_ANGLE_DEG = 3.0


def _clamp(value, low, high):
    return max(low, min(high, value))


class BallTaskController:
    """Run the complete task with a small, directly tunable action table."""

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
        self.reference_velocity_mm_s = 0.0
        self.reference_acceleration_mm_s2 = 0.0
        self.requested_beam_angle_deg = 0.0
        self.motor_target_pulses = 0
        self.active_drive_limit_pulses = beam_angle_to_motor_pulses(
            MAX_BEAM_ANGLE_DEG
        )
        self.stiction_active = False
        self.stiction_event_count = 0

    def force_fault(self, reason):
        self.state = "FAULT"
        self.fault_reason = str(reason)
        self.target_mm = 0.0
        self.target_goal_mm = 0.0
        self.requested_beam_angle_deg = 0.0
        self.motor_target_pulses = 0
        self.active_drive_limit_pulses = 0
        self.stiction_active = False

    def _enter(self, state, now):
        self.state = str(state)
        self.state_since = now
        self.settle_since = None
        self.stiction_active = False

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
                    self.start_time = now
                    self._enter("HOLD_CENTER", now)
            else:
                self.arm_since = None
            return

        if not valid:
            self.force_fault("vision timeout")
            return
        if abs(position_mm) >= BALL_EDGE_LIMIT_MM:
            self.force_fault("ball edge")
            return
        if now - self.start_time >= TASK_TIME_LIMIT_S:
            self.force_fault("task timeout")
            return

        phase_time_s = now - self.state_since

        if self.state == "HOLD_CENTER":
            if phase_time_s >= CENTER_HOLD_TIME_S:
                self._enter("POS_PUSH", now)
            return

        if self.state == "POS_PUSH":
            should_brake = (
                phase_time_s >= POSITIVE_PUSH_MIN_TIME_S
                and (
                    position_mm >= POSITIVE_BRAKE_POSITION_MM
                    or velocity_mm_s >= POSITIVE_BRAKE_SPEED_MM_S
                )
            )
            if should_brake or phase_time_s >= POSITIVE_PUSH_MAX_TIME_S:
                self._enter("POS_BRAKE", now)
            return

        if self.state == "POS_BRAKE":
            ready_to_settle = (
                phase_time_s >= POSITIVE_BRAKE_MIN_TIME_S
                and position_mm >= POSITIVE_SETTLE_ENTRY_MM
                and abs(velocity_mm_s) <= POSITIVE_SETTLE_ENTRY_SPEED_MM_S
            )
            if ready_to_settle:
                self._enter("POS_SETTLE", now)
            elif phase_time_s >= POSITIVE_BRAKE_MAX_TIME_S:
                self._enter("POS_SETTLE", now)
            return

        if self.state == "POS_SETTLE":
            # Do not reverse merely on crossing +50 mm at high speed.  The
            # trial-13 plant crossed at 120 mm/s and coasted to +70.7 mm.
            # Keep the stronger positive settle brake until both scoring
            # position and a bounded reversal speed are observed.
            if (
                abs(position_mm - POSITIVE_TARGET_MM)
                <= POSITIVE_SETTLE_ERROR_MM
                and abs(velocity_mm_s) <= 40.0
            ):
                self._enter("NEG_PUSH", now)
                return
            return

        if self.state == "NEG_PUSH":
            should_brake = (
                phase_time_s >= NEGATIVE_PUSH_MIN_TIME_S
                and (
                    position_mm <= NEGATIVE_BRAKE_POSITION_MM
                    or velocity_mm_s <= NEGATIVE_BRAKE_SPEED_MM_S
                    or (
                        position_mm
                        <= NEGATIVE_PREDICTIVE_BRAKE_POSITION_MM
                        and velocity_mm_s
                        <= NEGATIVE_PREDICTIVE_BRAKE_SPEED_MM_S
                    )
                )
            )
            if should_brake or phase_time_s >= NEGATIVE_PUSH_MAX_TIME_S:
                self._enter("NEG_BRAKE", now)
            return

        if self.state == "NEG_BRAKE":
            ready_to_settle = (
                phase_time_s >= NEGATIVE_BRAKE_MIN_TIME_S
                and (
                    position_mm <= NEGATIVE_SETTLE_ENTRY_MM
                    or velocity_mm_s >= NEGATIVE_BRAKE_EARLY_EXIT_SPEED_MM_S
                )
            )
            if (
                ready_to_settle
                or phase_time_s >= NEGATIVE_BRAKE_MAX_TIME_S
            ):
                self._enter("NEG_SETTLE", now)
            return

        if self.state == "NEG_SETTLE":
            if (
                abs(position_mm - NEGATIVE_TARGET_MM)
                <= FINAL_SETTLE_ERROR_MM
                and abs(velocity_mm_s) <= FINAL_SETTLE_SPEED_MM_S
            ):
                if FINAL_SETTLE_TIME_S <= 0.0:
                    self._enter("COMPLETE", now)
                elif self.settle_since is None:
                    self.settle_since = now
                elif now - self.settle_since >= FINAL_SETTLE_TIME_S:
                    self._enter("COMPLETE", now)
            else:
                self.settle_since = None

    def _settle_angle(
        self,
        target_mm,
        position_mm,
        velocity_mm_s,
        stiction_min_angle_deg=SETTLE_STICTION_MIN_ANGLE_DEG,
    ):
        error_mm = target_mm - position_mm
        angle_deg = (
            SETTLE_POSITION_KP_DEG_PER_MM * error_mm
            - SETTLE_VELOCITY_KD_DEG_PER_MM_S * velocity_mm_s
        )
        angle_deg = _clamp(
            angle_deg,
            -SETTLE_MAX_ANGLE_DEG,
            SETTLE_MAX_ANGLE_DEG,
        )
        self.stiction_active = False
        if (
            abs(error_mm) >= SETTLE_STICTION_ERROR_MM
            and abs(velocity_mm_s) <= SETTLE_STICTION_SPEED_MM_S
            and abs(angle_deg) < stiction_min_angle_deg
        ):
            angle_deg = (
                stiction_min_angle_deg
                if error_mm > 0.0
                else -stiction_min_angle_deg
            )
            self.stiction_active = True
            self.stiction_event_count += 1
        return angle_deg

    def _command_angle(self, position_mm, velocity_mm_s):
        self.stiction_active = False
        if self.state == "POS_PUSH":
            phase_time_s = 0.0
            if (
                self.last_update_time is not None
                and self.state_since is not None
            ):
                phase_time_s = self.last_update_time - self.state_since
            if (
                phase_time_s >= POSITIVE_BREAKAWAY_DELAY_S
                and abs(position_mm) <= POSITIVE_BREAKAWAY_POSITION_MM
                and abs(velocity_mm_s) <= POSITIVE_BREAKAWAY_SPEED_MM_S
            ):
                return POSITIVE_BREAKAWAY_ANGLE_DEG
            return POSITIVE_PUSH_ANGLE_DEG
        if self.state == "POS_BRAKE":
            return POSITIVE_BRAKE_ANGLE_DEG
        if self.state == "NEG_PUSH":
            return NEGATIVE_PUSH_ANGLE_DEG
        if self.state == "NEG_BRAKE":
            return NEGATIVE_BRAKE_ANGLE_DEG
        if self.state == "POS_SETTLE":
            return self._settle_angle(
                POSITIVE_TARGET_MM,
                position_mm,
                velocity_mm_s,
                POSITIVE_SETTLE_STICTION_MIN_ANGLE_DEG,
            )
        if self.state == "NEG_SETTLE":
            return self._settle_angle(
                NEGATIVE_TARGET_MM,
                position_mm,
                velocity_mm_s,
            )
        if self.state == "COMPLETE":
            return self._settle_angle(
                NEGATIVE_TARGET_MM,
                position_mm,
                velocity_mm_s,
                0.0,
            )
        if self.state == "HOLD_CENTER":
            return self._settle_angle(0.0, position_mm, velocity_mm_s)
        return 0.0

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

        if self.state in ("POS_PUSH", "POS_BRAKE", "POS_SETTLE"):
            self.target_mm = POSITIVE_TARGET_MM
            self.target_goal_mm = POSITIVE_TARGET_MM
        elif self.state in (
            "NEG_PUSH",
            "NEG_BRAKE",
            "NEG_SETTLE",
            "COMPLETE",
        ):
            self.target_mm = NEGATIVE_TARGET_MM
            self.target_goal_mm = NEGATIVE_TARGET_MM
        else:
            self.target_mm = 0.0
            self.target_goal_mm = 0.0

        if self.state not in (
            "WAIT_CENTER",
            "PIXEL_CALIBRATION",
            "FAULT",
        ) and valid:
            self.requested_beam_angle_deg = _clamp(
                self._command_angle(position_mm, velocity_mm_s),
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
            self.requested_beam_angle_deg = 0.0
            self.motor_target_pulses = 0
            self.stiction_active = False

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
            "reference_velocity_mm_s": 0.0,
            "reference_acceleration_mm_s2": 0.0,
            "desired_acceleration_mm_s2": 0.0,
            "requested_beam_angle_deg": self.requested_beam_angle_deg,
            "integral_angle_deg": 0.0,
            "stiction_active": self.stiction_active,
            "stiction_stationary_ms": 0.0,
            "stiction_event_count": self.stiction_event_count,
            "vision_reject_count": (
                self.estimator.rejected_measurement_count
            ),
            "motor_target_pulses": self.motor_target_pulses,
            "drive_limit_pulses": self.active_drive_limit_pulses,
            "elapsed_s": elapsed_s,
            "dt_s": dt,
        }
