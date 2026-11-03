"""Model-neutral detector contract; no guessed kmodel tensor format or fake results."""
import os
from config import BOMB_LABELS, DETECTION_SCORE_MIN, MAX_DETECTIONS
from geometry import finite


def _failure(reason):
    return {"ok": False, "reason": reason, "detections": []}


def restore_bbox_xywh(box, transform, image_size):
    """Inverse of actual raw -> network resize and padding, including rounding."""
    if len(box) != 4 or not all(finite(v) for v in box) or min(box[2:]) <= 0:
        raise ValueError("BAD_DETECTION_BOX")
    if tuple(transform["raw_size"]) != tuple(image_size):
        raise ValueError("PREPROCESS_RAW_SIZE_MISMATCH")
    sx, sy = transform["scale_x"], transform["scale_y"]
    px, py = transform["pad_x"], transform["pad_y"]
    if not all(finite(v) for v in (sx, sy, px, py)) or min(sx, sy) <= 0:
        raise ValueError("BAD_PREPROCESS_TRANSFORM")
    x, y, w, h = box
    x0, y0 = (x-px)/sx, (y-py)/sy
    x1, y1 = (x+w-px)/sx, (y+h-py)/sy
    width, height = image_size
    clipped = x0 < 0 or y0 < 0 or x1 > width or y1 > height
    x0, y0, x1, y1 = max(0.0, x0), max(0.0, y0), min(float(width), x1), min(float(height), y1)
    if x1 <= x0 or y1 <= y0:
        raise ValueError("DETECTION_IN_PADDING_ONLY")
    return (x0, y0, x1-x0, y1-y0), clipped


class YoloDetector:
    """Backend load(metadata), infer(raw) and close() are supplied with the model.

    infer returns {space, detections, transform}. space is raw_pixels or
    network_pixels; each box is xywh. Backend owns preprocessing AND NMS.
    transform is mandatory for network pixels, and describes the operation
    actually performed, not an idealized width/height ratio.
    """
    def __init__(self, metadata, backend=None, allow_test_backend=False,
                 labels=None, id_field="color_id", source_name="yolo"):
        self.metadata, self.backend = metadata, backend
        self.labels = tuple(BOMB_LABELS if labels is None else labels)
        if not self.labels or len(self.labels) > 32 or len(set(self.labels)) != len(self.labels) or any(not isinstance(v, str) or not v for v in self.labels):
            raise ValueError("INVALID_MODEL_LABELS")
        if id_field not in ("color_id", "shape_id") or not isinstance(source_name, str) or not source_name:
            raise ValueError("INVALID_DETECTOR_SEMANTICS")
        self.id_field, self.source_name = id_field, source_name
        self.ready, self.reason = False, "MODEL_NOT_READY"
        self.allow_test_backend = allow_test_backend

    def load(self):
        self.close()
        path = self.metadata.get("model_path")
        if not path:
            self.reason = "MODEL_NOT_READY"
            return _failure(self.reason)
        try:
            os.stat(path)
        except OSError:
            self.reason = "MODEL_NOT_READY"
            return _failure(self.reason)
        required = ("family", "input_size", "input_dtype", "tensor_layout", "color_order",
                    "normalization", "preprocess", "output_contract", "nncase_version")
        if any(self.metadata.get(k) is None for k in required) or self.metadata.get("contract_verified") is not True:
            self.reason = "MODEL_METADATA_NOT_VERIFIED"
            return _failure(self.reason)
        if tuple(self.metadata.get("labels", ())) != self.labels:
            self.reason = "MODEL_CLASS_ORDER_MISMATCH"
            return _failure(self.reason)
        if self.backend is None:
            self.reason = "MODEL_BACKEND_NOT_CONFIGURED"
            return _failure(self.reason)
        if getattr(self.backend, "is_test_backend", False) and not self.allow_test_backend:
            self.reason = "TEST_BACKEND_DISABLED"
            return _failure(self.reason)
        try:
            result = self.backend.load(self.metadata)
            if not result or result.get("ok") is not True:
                self.reason = "MODEL_LOAD_FAILED"
                self.backend.close()
                return _failure(self.reason)
        except Exception:
            self.reason = "MODEL_LOAD_FAILED"
            self.backend.close()
            return _failure(self.reason)
        self.ready, self.reason = True, "OK"
        return {"ok": True, "reason": "OK"}

    def detect(self, raw):
        if not self.ready:
            return _failure(self.reason)
        try:
            size = (raw.width(), raw.height())
            if size != tuple(self.metadata["raw_size"]):
                return _failure("IMAGE_SIZE_MISMATCH")
            output = self.backend.infer(raw)
            items = output["detections"]
            if len(items) > MAX_DETECTIONS:
                return _failure("TOO_MANY_DETECTIONS")
            detections = []
            for item in items:
                cls, score = item["class_id"], item["score"]
                if type(cls) is not int or not 0 <= cls < len(self.labels) or not finite(score) or not 0 <= score <= 1:
                    raise ValueError("INVALID_DETECTION_CLASS_OR_SCORE")
                if score < DETECTION_SCORE_MIN:
                    continue
                if output["space"] == "network_pixels":
                    box, clipped = restore_bbox_xywh(item["bbox_xywh"], output["transform"], size)
                elif output["space"] == "raw_pixels":
                    identity = {"raw_size": size, "scale_x": 1.0, "scale_y": 1.0, "pad_x": 0.0, "pad_y": 0.0}
                    box, clipped = restore_bbox_xywh(item["bbox_xywh"], identity, size)
                else:
                    raise ValueError("UNKNOWN_DETECTION_COORDINATES")
                detections.append({"class_id": cls, "label": self.labels[cls], self.id_field: cls+1,
                                   "score": score, "bbox_raw_px": box, "bbox_clipped": clipped})
            return {"ok": True, "reason": "OK", "detections": detections,
                    "source": "test_backend" if getattr(self.backend, "is_test_backend", False) else self.source_name}
        except Exception as exc:
            return _failure("DETECTION_ERROR:"+str(exc))

    def close(self):
        if self.ready and self.backend is not None:
            self.backend.close()
        self.ready = False
        self.reason = "MODEL_NOT_READY"
