from boundary_policy import enabled as boundary_checks
"""Target slot scan and gripper-frame center observation; no firing or UART."""
import aim_config as settings
from config import IMAGE_SIZE, GEOMETRY_ID, COLOR_NAMES, REPORT_CANDIDATE_TRANSFORMS
from geometry import norm, finite
from target_pose import fit_target_center
from target_color import (ColorTargetDetector, extract_target_rings, valid_color,
                          target_area_rejection, target_priority)
from tasks.task_bomb import BombScanSession


def _error(reason, frame_id=None):
    return {"ok": False, "reason": reason, "frame_id": frame_id,
            "position_camera_mm": None, "position_tool_mm": None,
            "orientation_valid": False, "control_ready": False,
            "laser_offset_compensated": False, "hit_confirmed": False}


def measure_target_from_contours(calibration, outer_points_px, inner_points_px,
                                 color_id, frame_id, geometry_confirmed=False,
                                 stable=False, source="external_contours"):
    if not valid_color(color_id):
        return _error("INVALID_COLOR_ID", frame_id)
    if not geometry_confirmed:
        return _error("RAW_GEOMETRY_NOT_CONFIRMED", frame_id)
    if not stable:
        return _error("OBSERVATION_NOT_STABLE", frame_id)
    result = fit_target_center(calibration.camera, outer_points_px, inner_points_px,
                               settings.TARGET_OUTER_DIAMETER_MM, settings.TARGET_INNER_DIAMETER_MM)
    if not result["ok"]:
        return _error(result["reason"], frame_id)
    transformed = calibration.point_to_tool(result["position_camera_mm"], REPORT_CANDIDATE_TRANSFORMS)
    result.update(frame_id=frame_id, color_id=color_id, color=COLOR_NAMES[color_id],
                  geometry_valid=True, geometry_id=GEOMETRY_ID,
                  position_tool_mm=transformed["position_tool_mm"],
                  handeye_accepted=calibration.handeye_accepted,
                  coordinate_frame="E_commanded_gripper_center", units="mm",
                  laser_offset_compensated=False, hit_confirmed=False, source=source,
                  control_ready=transformed["control_ready"] and source in ("color_target", "yolo_target"))
    result["ok"], result["reason"] = transformed["ok"], transformed["reason"]
    return result


class TargetScanSession(BombScanSession):
    """Reuse the verified three-color sequence logic; only target gate differs."""
    def __init__(self, target_color_id, scan_order_confirmed=False,
                 start_boundary_confirmed=False, gate=None):
        super().__init__(target_color_id, scan_order_confirmed, start_boundary_confirmed,
                         gate or settings.SCAN_GATE_RAW_PX,
                         settings.SCAN_MIN_HITS, settings.SCAN_RELEASE_FRAMES)


class AimTask:
    def __init__(self, calibration, detector=None, measure_hits=3, max_scatter_mm=6.0):
        if type(measure_hits) is not int or not 1 <= measure_hits <= 10 or not finite(max_scatter_mm) or max_scatter_mm <= 0:
            raise ValueError("INVALID_MEASUREMENT_STABILITY")
        self.calibration = calibration
        self.detector = detector if detector is not None else ColorTargetDetector()
        self.measure_hits, self.max_scatter_mm = measure_hits, max_scatter_mm
        self.mode, self.scan, self.samples = "IDLE", None, []
        self.last_frame, self.frames, self.max_frames = -1, 0, 0
        self.target_color, self.slot_id = None, None

    def start_scan(self, target_color_id, scan_order_confirmed=False,
                   start_boundary_confirmed=False, gate=None, max_frames=2400):
        self._budget(max_frames)
        self.scan = TargetScanSession(target_color_id, scan_order_confirmed, start_boundary_confirmed, gate)
        self.mode, self.samples, self.last_frame, self.frames = "SCAN", [], -1, 0

    def start_measure(self, color_id, slot_id=None, max_frames=180):
        if not valid_color(color_id) or (slot_id is not None and not valid_color(slot_id)):
            raise ValueError("INVALID_TARGET")
        self._budget(max_frames)
        self.target_color, self.slot_id = color_id, slot_id
        self.mode, self.samples, self.scan = "MEASURE", [], None
        self.last_frame, self.frames = -1, 0

    def _budget(self, max_frames):
        if type(max_frames) is not int or max_frames < 1:
            raise ValueError("INVALID_FRAME_BUDGET")
        self.max_frames = max_frames

    def _invalid(self, reason, frame_id):
        self.samples = []
        return _error(reason, frame_id)

    def poll(self, raw, frame_id, geometry_confirmed=False, stable=False):
        if self.mode not in ("SCAN", "MEASURE"):
            return _error("TASK_NOT_ACTIVE", frame_id)
        if type(frame_id) is not int or frame_id <= self.last_frame:
            return _error("STALE_OR_DUPLICATE_FRAME", frame_id)
        previous = self.last_frame
        self.last_frame, self.frames = frame_id, self.frames+1
        if self.frames > self.max_frames:
            self.mode, self.samples = "ERROR", []
            return _error("TARGET_TASK_FRAME_BUDGET_EXCEEDED", frame_id)
        if not geometry_confirmed or (raw.width(), raw.height()) != IMAGE_SIZE:
            if self.mode == "SCAN": self.scan.drop_frame("RAW_GEOMETRY_NOT_CONFIRMED")
            return self._invalid("RAW_GEOMETRY_NOT_CONFIRMED", frame_id)
        if self.mode == "MEASURE" and not stable:
            return self._invalid("OBSERVATION_NOT_STABLE", frame_id)
        try:
            detection = self.detector.detect(raw)
        except Exception as exc:
            if self.mode == "SCAN": self.scan.drop_frame("TARGET_DETECTOR_ERROR:"+str(exc))
            return self._invalid("TARGET_DETECTOR_ERROR:"+str(exc), frame_id)
        if not detection["ok"]:
            if self.mode == "SCAN": self.scan.drop_frame(detection["reason"])
            return self._invalid(detection["reason"], frame_id)
        if self.mode == "SCAN":
            result = self.scan.update(detection["detections"], frame_id)
            result.update(frame_id=frame_id, task="aim", laser_offset_compensated=False)
            if result["complete"]: self.mode = "DONE"
            return result
        matches = [d for d in detection["detections"] if d["color_id"] == self.target_color]
        if not matches:
            reasons = [r["reason"] for r in detection.get("rejected", []) if r.get("color_id") == self.target_color]
            return self._invalid(reasons[0] if reasons else "TARGET_NOT_FOUND", frame_id)
        x,y,w,h=settings.TARGET_DETECTION_ROI_RAW_PX
        matches=[item for item in matches if
                 x<=item["bbox_raw_px"][0]+item["bbox_raw_px"][2]*.5<x+w and
                 y<=item["bbox_raw_px"][1]+item["bbox_raw_px"][3]*.5<y+h]
        if not matches:
            return self._invalid("TARGET_OUTSIDE_DETECTION_ROI",frame_id)
        # Multiple boxes are candidates, not a measurement failure. Keep the
        # requested color/working region, then try the largest valid annulus.
        rejected=[]
        candidates=[]
        for target in matches:
            reason=target_area_rejection(target)
            if reason:rejected.append({"bbox_raw_px":target["bbox_raw_px"],"reason":reason})
            else:candidates.append(target)
        candidates.sort(key=target_priority)
        result=None
        failure_reason=None
        rgb=None
        try:
            for target in candidates:
                reason=None
                if boundary_checks() and target.get("bbox_clipped",False):
                    reason="TARGET_CONTOUR_CLIPPED"
                contours=target.get("contours")
                if reason is None and contours is None:
                    if rgb is None:rgb=raw.to_rgb565()
                    contours=extract_target_rings(rgb,target["bbox_raw_px"],self.target_color)
                if reason is None:
                    reason=(contours.get("reason","TARGET_RINGS_TOO_SMALL") if not contours["ok"]
                            else target_area_rejection(target,contours))
                if reason is None:
                    measured=measure_target_from_contours(
                        self.calibration,contours["outer_points_px"],contours["inner_points_px"],
                        self.target_color,frame_id,True,True,detection.get("source","unknown"))
                    if measured["ok"]:
                        result=measured
                        result.update(selected_target_bbox_raw_px=target["bbox_raw_px"],
                                      selected_target_area_px=target["bbox_raw_px"][2]*target["bbox_raw_px"][3],
                                      target_candidate_count=len(matches),candidate_rejections=rejected)
                        break
                    reason=measured["reason"]
                if failure_reason is None:failure_reason=reason
                rejected.append({"bbox_raw_px":target["bbox_raw_px"],"reason":reason})
        finally:
            rgb=None
        if result is None:
            reason=failure_reason or (rejected[0]["reason"] if rejected else "TARGET_NOT_FOUND")
            failure=self._invalid(reason,frame_id)
            failure.update(target_candidate_count=len(matches),candidate_rejections=rejected)
            return failure
        if previous >= 0 and frame_id != previous+1:
            self.samples = []
        self.samples.append((frame_id, result["position_camera_mm"]))
        self.samples = self.samples[-self.measure_hits:]
        mean = [sum(s[1][i] for s in self.samples)/len(self.samples) for i in range(3)]
        scatter = max(norm([s[1][i]-mean[i] for i in range(3)]) for s in self.samples)
        if scatter > self.max_scatter_mm:
            self.samples = self.samples[-1:]
            return _error("TARGET_POSITION_NOT_STABLE", frame_id)
        if len(self.samples) < self.measure_hits:
            return _error("TARGET_MEASUREMENT_STABILIZING", frame_id)
        result["position_camera_mm"] = mean
        result["position_tool_mm"] = self.calibration.point_to_tool(mean, REPORT_CANDIDATE_TRANSFORMS)["position_tool_mm"]
        result.update(slot_id=self.slot_id, sample_frame_ids=[s[0] for s in self.samples],
                      measurement_scatter_mm=scatter, measurement_complete=True)
        self.samples, self.mode = [], "DONE"
        return result

    def finish_scan(self):
        if self.scan is None or self.mode not in ("SCAN", "DONE"):
            return _error("SCAN_NOT_ACTIVE")
        self.mode = "DONE"
        return self.scan.finish()

    def cancel(self):
        self.mode, self.scan, self.samples = "CANCELLED", None, []

    def close(self):
        self.cancel()
        self.detector.close()
