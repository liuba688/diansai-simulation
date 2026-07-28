"""MaixCAM steel-ball car configuration.

Keep field-tunable values in this file. The detector threshold is deliberately
lower than the control threshold so weak detections remain visible without
making the car leave the line.
"""

MODEL_FILE_NAME = "steel_ball_320_v2.mud"
MODEL_BINARY_FILE_NAME = "steel_ball_320_v2.cvimodel"

DETECTOR_CONFIDENCE_THRESHOLD = 0.45
CANDIDATE_CONFIDENCE_THRESHOLD = 0.45
DISPLAY_CONFIDENCE_THRESHOLD = 0.50
CONTROL_CONFIDENCE_THRESHOLD = 0.60
IOU_THRESHOLD = 0.35

MIN_ASPECT_RATIO = 0.70
MAX_ASPECT_RATIO = 1.0 / MIN_ASPECT_RATIO
MIN_BOX_SIDE = 8

TRACK_CONFIRM_FRAMES = 3
CONTROL_CONFIRM_FRAMES = 5
TRACK_MAX_MISSES = 2
TRACK_MATCH_DISTANCE_FACTOR = 1.25
TRACK_SMOOTHING = 0.60

# The initial close-distance estimate. Calibrate this on the real car by
# observing the box height just before the ball enters the pickup blind zone.
CLOSE_BOX_HEIGHT_RATIO = 0.30

UART_ENABLED = True
UART_DEVICE = "/dev/ttyS1"
UART_BAUDRATE = 115200
UART_TX_PIN = "A19"
UART_RX_PIN = "A18"
UART_TX_FUNCTION = "UART1_TX"
UART_RX_FUNCTION = "UART1_RX"
UART_SEND_INTERVAL_SECONDS = 1.0 / 30.0
UART_STATUS_POLL_INTERVAL_SECONDS = 0.05

LOG_INTERVAL_SECONDS = 0.5
