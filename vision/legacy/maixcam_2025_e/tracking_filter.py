"""目标滤波和安全状态。"""

import config


STATUS_LOST = 0
STATUS_TRACKING = 1
STATUS_HOLD = 2
STATUS_SEARCH = 3


class TrackState:
    def __init__(self):
        self.status = STATUS_LOST
        self.valid = False
        self.fresh = False
        self.center_x = 0
        self.center_y = 0
        self.width = 0
        self.height = 0
        self.dx = 0
        self.dy = 0
        self.confidence = 0
        self.corners = None
        self.missed_frames = 0


class TargetTracker:
    def __init__(self):
        self._initialized = False
        self._center_x = 0.0
        self._center_y = 0.0
        self._width = 0.0
        self._height = 0.0
        self._confidence = 0.0
        self._corners = None
        self._missed_frames = 0

    def _filter(self, old_value, new_value):
        return old_value + config.TRACK_EMA_ALPHA * (new_value - old_value)

    def update(self, detection):
        if detection.found and not detection.precise:
            self._missed_frames += 1
            if self._missed_frames >= config.LOST_FRAMES:
                self._initialized = False
            state = TrackState()
            state.status = STATUS_SEARCH
            state.valid = True
            state.fresh = True
            state.center_x, state.center_y = detection.center
            state.width = detection.width
            state.height = detection.height
            state.dx = state.center_x - config.AIM_REFERENCE_X
            state.dy = state.center_y - config.AIM_REFERENCE_Y
            state.confidence = detection.confidence
            state.corners = detection.corners
            state.missed_frames = self._missed_frames
            return state

        if detection.found:
            if not self._initialized:
                self._center_x = float(detection.center[0])
                self._center_y = float(detection.center[1])
                self._width = float(detection.width)
                self._height = float(detection.height)
                self._confidence = float(detection.confidence)
                self._initialized = True
            else:
                self._center_x = self._filter(self._center_x, detection.center[0])
                self._center_y = self._filter(self._center_y, detection.center[1])
                self._width = self._filter(self._width, detection.width)
                self._height = self._filter(self._height, detection.height)
                self._confidence = self._filter(
                    self._confidence, detection.confidence
                )
            self._corners = detection.corners
            self._missed_frames = 0
            return self._make_state(STATUS_TRACKING, True)

        self._missed_frames += 1
        if self._initialized and self._missed_frames <= config.HOLD_FRAMES:
            return self._make_state(STATUS_HOLD, False)
        if self._missed_frames >= config.LOST_FRAMES:
            self._initialized = False
        state = TrackState()
        state.missed_frames = self._missed_frames
        return state

    def _make_state(self, status, fresh):
        state = TrackState()
        state.status = status
        state.valid = True
        state.fresh = fresh
        state.center_x = int(round(self._center_x))
        state.center_y = int(round(self._center_y))
        state.width = int(round(self._width))
        state.height = int(round(self._height))
        state.dx = state.center_x - config.AIM_REFERENCE_X
        state.dy = state.center_y - config.AIM_REFERENCE_Y
        state.confidence = max(0, min(100, int(round(self._confidence))))
        state.corners = self._corners
        state.missed_frames = self._missed_frames
        return state
