"""Target task settings. Initial color thresholds require field adjustment."""
TARGET_OUTER_DIAMETER_MM = 100.0
TARGET_INNER_DIAMETER_MM = 55.0
# Zero-based mask order in CanMV find_blobs: red, green, blue.
LAB_THRESHOLDS = ((36, 52, 36, 59, 7, 37),
                  (25, 66, -44, -8, 7, 47),
                  (19, 61, -4, 25, -58, -14))
HUE_DEG = {1: ((0.0, 25.0), (335.0, 360.0)),
           2: ((70.0, 170.0),), 3: ((180.0, 265.0),)}
MIN_SATURATION = 0.30
MIN_VALUE = 0.12
MAX_CANDIDATES = 9
MAX_ROI_CELLS = 10000
MAX_POINTS_PER_RING = 16
MIN_COMPONENT_SAMPLES = 45
MIN_HOLE_SAMPLES = 16
HOLE_AREA_RATIO = (0.12, 0.55)
MAX_REPROJECTION_PX = 3.0
MIN_INLIER_RATIO = 0.85
MIN_COVERAGE_DEG = 275.0
MIN_OUTER_RADIUS_PX = 20.0
MIN_DEPTH_MM = 100.0
MAX_DEPTH_MM = 1600.0
CORNER_NOISE_PX = 0.8  # Assumed contour noise; not a measured confidence interval.
MAX_POSITION_3SIGMA_MM = 300.0
# Original 640x480 pixels: center (320,240), 160x160 center-eligibility region.
TARGET_DETECTION_ROI_RAW_PX = (250, 170, 150, 150)
SCAN_GATE_RAW_PX = TARGET_DETECTION_ROI_RAW_PX
SCAN_MIN_HITS = 3
SCAN_RELEASE_FRAMES = 2
# Ignoring laser installation offset is intentional for this delivery.
LASER_OFFSET_COMPENSATED = False
