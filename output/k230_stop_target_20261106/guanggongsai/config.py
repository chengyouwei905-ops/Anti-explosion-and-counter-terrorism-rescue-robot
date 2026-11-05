"""K230 vision only. No UART, motors, or hardware initialization on import."""
IMAGE_SIZE = (640, 480)
# False: contour touching/coverage/pixel residuals are diagnostics, not rejection gates.
BOUNDARY_CHECKS_ENABLED = False
GEOMETRY_ID = "sensor0_vga_rgb888_original_calibration_20261023"
QR_SIZE_MM = 80.0
# Only center consistency and position uncertainty are accepted, both in mm.
QR_MAX_CENTER_DISAGREEMENT_MM = 12.0
QR_MAX_POSITION_3SIGMA_MM = 50.0
BOMB_DIAMETER_MM = 40.0
# YOLO class IDs are zero-based; competition color IDs are one-based.
BOMB_LABELS = ("red_bomb", "green_bomb", "blue_bomb")
COLOR_NAMES = (None, "red", "green", "blue")
MODEL_CONFIG_PATH = "models/bomb_three_colors.template.json"
DETECTION_SCORE_MIN = 0.55
MAX_DETECTIONS = 12
# Initial HSV fallback for colors without measured LAB thresholds below.
COLOR_HUE_DEG = {1: ((0.0, 25.0), (335.0, 360.0)),
                 2: ((70.0, 170.0),), 3: ((180.0, 265.0),)}
COLOR_MIN_SATURATION = 0.30
COLOR_MIN_VALUE = 0.12
# 用户现场提供，2026-11-03；只用于红色爆炸物轮廓分割，不改变打靶阈值。
BOMB_LAB_THRESHOLDS = {1: (26, 54, 28, 75, 18, 46), 2: (35, 71, -52, -14, 17, 58), 3: (21, 38, 1, 35, -69, -5)}
CONTOUR_MAX_CELLS = 12000
CONTOUR_MAX_POINTS = 48
CONTOUR_MIN_PIXELS = 40
SPHERE_MAX_ERROR_PX = 10.0
SPHERE_MIN_INLIER_RATIO = 0.80
SPHERE_MIN_COVERAGE_DEG = 270.0
SPHERE_MIN_RADIUS_PX = 20.0
SPHERE_MAX_DISTANCE_MM = 2000.0
SCAN_MIN_HITS = 3
SCAN_RELEASE_FRAMES = 2
# VGA 640x480的居中400x480工作区域：(x,y,width,height)。
# 同时约束扫描计数与球心定位；YOLO物体框仍随目标实际大小变化。
BOMB_DETECTION_ROI_RAW_PX = (120, 0, 400, 480)
SCAN_GATE_RAW_PX = BOMB_DETECTION_ROI_RAW_PX
# 检测框仅是轮廓计算的起始区域，碰边时允许一直扩展到完整原图。
BOMB_FULL_FRAME_SEARCH = True
SCAN_MAX_STEP_PX = 60.0
# Candidates may be inspected; this never changes the calibration acceptance flag.
REPORT_CANDIDATE_TRANSFORMS = True
