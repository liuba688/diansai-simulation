"""Three-point PPR beam calibration and robust ball velocity estimate."""

POSITIVE_50_PX = 212.0
CENTER_0_PX = 324.0
NEGATIVE_50_PX = 435.0
VISION_TIMEOUT_S = 0.150
VELOCITY_ALPHA = 0.35
OUTLIER_MIN_JUMP_MM = 12.0
OUTLIER_MAX_SPEED_MM_S = 300.0


def clamp(value, low, high):
    return max(low, min(high, value))


def pixel_to_mm(pixel_x):
    pixel_x = float(pixel_x)
    if pixel_x <= CENTER_0_PX:
        return (CENTER_0_PX - pixel_x) * 50.0 / (CENTER_0_PX - POSITIVE_50_PX)
    return -(pixel_x - CENTER_0_PX) * 50.0 / (NEGATIVE_50_PX - CENTER_0_PX)


class BallEstimator:
    def __init__(self):
        self.reset()

    def reset(self):
        self.valid = False
        self.position_mm = 0.0
        self.velocity_mm_s = 0.0
        self.measurement_time = 0.0
        self.confidence = 0.0
        self.outlier_mm = None
        self.outlier_count = 0

    def update(self, pixel_x, confidence, now):
        measured = pixel_to_mm(pixel_x)
        if self.valid:
            dt = now - self.measurement_time
            maximum_jump = max(OUTLIER_MIN_JUMP_MM,
                               OUTLIER_MAX_SPEED_MM_S * max(0.0, dt))
            if 0.003 <= dt <= 0.200 and abs(measured - self.position_mm) > maximum_jump:
                if self.outlier_mm is not None and abs(measured - self.outlier_mm) <= 8.0:
                    self.outlier_count += 1
                else:
                    self.outlier_mm = measured
                    self.outlier_count = 1
                if self.outlier_count < 2:
                    return False
                self.velocity_mm_s = 0.0
            elif 0.003 <= dt <= 0.200:
                measured_velocity = clamp((measured - self.position_mm) / dt,
                                          -1000.0, 1000.0)
                self.velocity_mm_s = ((1.0 - VELOCITY_ALPHA) * self.velocity_mm_s
                                      + VELOCITY_ALPHA * measured_velocity)
            else:
                self.velocity_mm_s = 0.0
        else:
            self.valid = True
            self.velocity_mm_s = 0.0
        self.outlier_mm = None
        self.outlier_count = 0
        self.position_mm = measured
        self.measurement_time = now
        self.confidence = float(confidence)
        return True

    def estimate(self, now):
        if not self.valid:
            return 0.0, 0.0, 1000000000.0, False
        age = max(0.0, now - self.measurement_time)
        position = self.position_mm + self.velocity_mm_s * min(age, 0.050)
        return position, self.velocity_mm_s, age, age <= VISION_TIMEOUT_S
