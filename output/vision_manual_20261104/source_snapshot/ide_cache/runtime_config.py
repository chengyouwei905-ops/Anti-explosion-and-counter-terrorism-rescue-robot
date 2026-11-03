"""Single-entry runtime. MCU owns movement, physical stability and task order."""
PROJECT_ROOT = "/data/guanggongsai"
UART_ID = 2
UART_BAUDRATE = 115200
UART_TX_PIN = 11
UART_RX_PIN = 12
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
MEASUREMENT_SAMPLES = 3
MAX_POSITION_SCATTER_MM = 8.0
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
# 正式二维码：3秒滚动窗口累计5帧有效观测，允许期间短暂漏检/同码位置不确定。
# 仍使用MEASUREMENT_SAMPLES，与MCU的5帧验收一致；0恢复连续有效帧模式。
QR_SAMPLE_WINDOW_MS = 3000
# 原灰度全图先检测；连续3次未读出任何二维码后，在同尺寸副本上增强对比度重试。
QR_CONTRAST_RETRY_ENABLED = True
QR_CONTRAST_RETRY_EVERY_N_FRAMES = 3
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
# Slots are stable first-observation ordinals after the MCU scan command.
SCAN_REQUIRE_BLANK_GAP = False
SCAN_MAX_AMBIGUOUS_FRAMES = 3
RESCUE_REFERENCE_MODE = "object"

# Matches current MCU TOTALSTOP -> TASK_IDLE. Optional "pause" retains old job.
TOTAL_STOP_POLICY = "restart"
