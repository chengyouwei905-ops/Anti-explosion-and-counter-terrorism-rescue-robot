"""Single-entry runtime. MCU owns movement, physical stability and task order."""
PROJECT_ROOT = "/data/guanggongsai"
UART_ID = 3
UART_BAUDRATE = 115200
UART_TX_PIN = 50
UART_RX_PIN = 51
COMMAND_QUEUE_SIZE = 8
COMMANDS_PER_POLL = 8
RX_READ_BYTES = 128
# LCKFB user key: high when pressed. Boot waits for a fresh press.
USER_KEY_GPIO = 53
USER_KEY_ACTIVE = 1
USER_KEY_DEBOUNCE_MS = 40
USER_KEY_POLL_MS = 5
TOTAL_START_BYTES = b"TOTALSTART!"
TOTAL_STOP_BYTES = b"TOTALSTOP!"
LOOP_SLEEP_MS = 5
MEASUREMENT_SAMPLES = 5
MAX_POSITION_SCATTER_MM = 20.0
SETTLE_MS = 250
MEASUREMENT_TIMEOUT_MS = 60000
SCAN_TIMEOUT_MS = 60000
REQUEST_DEDUP_MS = 5000
REQUEST_HISTORY_SIZE = 16
# Declaration for the unchanged Sensor0 / VGA / RGB888 calibrated pipeline.
RAW_GEOMETRY_CONFIRMED = True
# 同一Sensor0完整视场：通道0保持640x480 RGB888供YOLO使用。
# 通道1只在二维码任务中读取1920x1080灰度图；两路均不单独裁剪/翻转。
# QRTask按X/Y各自倍率推导二维码内参，原内参、D和手眼矩阵均不修改。
QR_HIGH_RES_ENABLED = True
QR_CAPTURE_SIZE = (1920, 1080)
QR_CAPTURE_CHANNEL = 1
# QR检测门按640x480屏幕预览像素定义：中心200x200=(220,140,200,200)。
# 映射到1920x1080原图为(660,315,600,450)。SIZE=None恢复全图。
# REFERENCE_SIZE=None时改为按当前二维码原图像素定义SIZE。
QR_ROI_SIZE = (200, 200)
QR_ROI_REFERENCE_SIZE = (640, 480)
# 同一码有效定位在3秒窗口内累计3帧，容忍短暂漏检。
QR_SAMPLE_WINDOW_MS = 3000
# 原灰度ROI先检测；未读到码时在同尺寸副本上增强对比度，仍只检测相同ROI。
QR_CONTRAST_RETRY_ENABLED = True
QR_CONTRAST_RETRY_EVERY_N_FRAMES = 1
# Set true after validating a full monotonic pass from before slot 1.
SCAN_ORDER_CONFIRMED = True
SCAN_START_BOUNDARY_CONFIRMED = True
# Candidate hand-eye results may be inspected; protocol status is OBSERVATION_ONLY.
ALLOW_CANDIDATE_REPORTS = True
DISPLAY_ENABLED = True
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 480
DISPLAY_TO_IDE = False  # LCD output; enable explicitly when IDE mirroring is needed.
DISPLAY_REFRESH_MS = 100
SNAPSHOT_TIMEOUT_MS = 200
SNAPSHOT_ATTEMPTS = 3
MAX_CAMERA_RESTARTS = 2
CAMERA_RESTART_WAIT_MS = 200
CAMERA_WARMUP_FRAMES = 10

# A matching backend is supplied after the real model/export contract is known.
def create_model_backend(kind, metadata):
    from kpu_yolo_backend import KPUYoloBackend
    return KPUYoloBackend()

# User acceptance on 2026-10-28; original calibration result stays unchanged.
HANDEYE_ACCEPTED_BY_USER = True
# Four-byte scan requests use MCU-numbered stationary positions and stop at the
# selected QR target. Empty legacy requests retain first-observation ordinals.
SCAN_REQUIRE_BLANK_GAP = False
SCAN_MAX_AMBIGUOUS_FRAMES = 3
RESCUE_REFERENCE_MODE = "object"

# Matches current MCU TOTALSTOP -> TASK_IDLE. Optional "pause" retains old job.
TOTAL_STOP_POLICY = "restart"
