"""One three-class model; compatible with the existing detector backend contract."""
from yolo_detector import YoloDetector
from rescue_config import RESCUE_LABELS


class RescueYoloDetector(YoloDetector):
    def __init__(self, metadata, backend=None, allow_test_backend=False):
        super().__init__(metadata, backend, allow_test_backend,
                         labels=RESCUE_LABELS, id_field="shape_id", source_name="yolo_rescue")
