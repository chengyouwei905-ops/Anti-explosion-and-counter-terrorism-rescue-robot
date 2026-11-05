"""Sequential bomb scan and known-sphere refinement. No UART or motion commands."""
import math
import config
from geometry import finite, fit_sphere_center
from search_region import expand_to_frame
from boundary_policy import enabled as boundary_checks,visible_edges
from lab_color import matches_lab,rgb_to_lab_reference,native_lab_mask


def _error(reason, frame_id=None):
    return {"ok": False, "reason": reason, "frame_id": frame_id,
            "position_camera_mm": None, "position_tool_mm": None,
            "orientation_valid": False, "control_ready": False}


def _valid_color(color_id):
    return type(color_id) is int and 1 <= color_id <= 3


def _is_color(rgb, color_id):
    threshold=config.BOMB_LAB_THRESHOLDS.get(color_id)
    if threshold is not None:
        return matches_lab(rgb_to_lab_reference(rgb),threshold)
    r, g, b = [v/255.0 for v in rgb]
    maximum, minimum = max(r, g, b), min(r, g, b)
    delta = maximum-minimum
    if maximum < config.COLOR_MIN_VALUE or delta <= 0 or delta/maximum < config.COLOR_MIN_SATURATION:
        return False
    if maximum == r:
        hue = (60.0*(g-b)/delta) % 360.0
    elif maximum == g:
        hue = 60.0*(b-r)/delta+120.0
    else:
        hue = 60.0*(r-g)/delta+240.0
    return any(lo <= hue <= hi for lo, hi in config.COLOR_HUE_DEG[color_id])


def _neighbors(index, columns, rows):
    x, y = index % columns, index // columns
    if x:
        yield index-1
    if x+1 < columns:
        yield index+1
    if y:
        yield index-columns
    if y+1 < rows:
        yield index+columns


def extract_ball_boundary(rgb_image, bbox_raw_px, color_id, _bounds=None, _expansions=0):
    """Color-connected outer boundary on geometry-preserving RGB565 image.

    YOLO box is ROI only. Interior highlights are filled by an exterior flood
    fill; small disconnected patches are discarded. A clipped or ambiguous
    component is rejected, not converted into a precision bbox diameter.
    """
    try:
        if not _valid_color(color_id):
            raise ValueError("INVALID_COLOR_ID")
        width, height = rgb_image.width(), rgb_image.height()
        if (width, height) != config.IMAGE_SIZE:
            raise ValueError("IMAGE_SIZE_MISMATCH")
        if len(bbox_raw_px) != 4 or not all(finite(v) for v in bbox_raw_px) or min(bbox_raw_px[2:]) <= 0:
            raise ValueError("INVALID_ROI")
        x, y, w, h = bbox_raw_px
        margin = max(5, int(max(w, h)*0.15))
        if _bounds is None:
            x0, y0 = max(0, int(math.floor(x))-margin), max(0, int(math.floor(y))-margin)
            x1, y1 = min(width, int(math.ceil(x+w))+margin), min(height, int(math.ceil(y+h))+margin)
        else:
            x0,y0,x1,y1=_bounds
        if x1 <= x0 or y1 <= y0:
            raise ValueError("INVALID_ROI")
        step = 1
        while ((x1-x0+step-1)//step)*((y1-y0+step-1)//step) > config.CONTOUR_MAX_CELLS:
            step += 1
        columns, rows = (x1-x0+step-1)//step, (y1-y0+step-1)//step
        threshold=config.BOMB_LAB_THRESHOLDS.get(color_id)
        lab_mask=native_lab_mask(rgb_image,threshold) if threshold is not None else None
        mask = bytearray(columns*rows)
        for gy in range(rows):
            for gx in range(columns):
                pixel = (lab_mask if lab_mask is not None else rgb_image).get_pixel(
                    x0+gx*step, y0+gy*step, rgbtuple=True)
                if pixel is None or len(pixel) != 3:
                    raise ValueError("RGB_PIXEL_API_UNSUPPORTED")
                mask[gy*columns+gx] = int(any(pixel) if lab_mask is not None else _is_color(pixel, color_id))
        largest, second_size = [], 0
        for index in range(len(mask)):
            if mask[index] != 1:
                continue
            queue, head = [index], 0
            mask[index] = 2
            while head < len(queue):
                current = queue[head]
                head += 1
                for adjacent in _neighbors(current, columns, rows):
                    if mask[adjacent] == 1:
                        mask[adjacent] = 2
                        queue.append(adjacent)
            if len(queue) > len(largest):
                second_size, largest = max(second_size, len(largest)), queue
            else:
                second_size = max(second_size, len(queue))
        if len(largest) < config.CONTOUR_MIN_PIXELS:
            raise ValueError("COLOR_COMPONENT_TOO_SMALL")
        if second_size > len(largest)*0.35:
            raise ValueError("MULTIPLE_COLOR_COMPONENTS")
        if any(i % columns in (0, columns-1) or i // columns in (0, rows-1) for i in largest):
            bounds=(x0,y0,x1,y1)
            expanded=expand_to_frame(bounds,width,height)
            if config.BOMB_FULL_FRAME_SEARCH and expanded!=bounds:
                return extract_ball_boundary(rgb_image,bbox_raw_px,color_id,expanded,_expansions+1)
            if boundary_checks():raise ValueError("CONTOUR_CLIPPED")
        solid = bytearray(columns*rows)
        for i in largest:
            solid[i] = 1
        queue, head = [], 0
        for i in range(len(solid)):
            if solid[i] == 0 and (i % columns in (0, columns-1) or i // columns in (0, rows-1)):
                solid[i] = 2
                queue.append(i)
        while head < len(queue):
            current = queue[head]
            head += 1
            for adjacent in _neighbors(current, columns, rows):
                if solid[adjacent] == 0:
                    solid[adjacent] = 2
                    queue.append(adjacent)
        points = []
        for i in largest:
            gx, gy = i % columns, i // columns
            px, py = x0+gx*step, y0+gy*step
            for adjacent, dx, dy in visible_edges(i,columns,rows):
                if solid[adjacent] == 2:
                    points.append((px+dx*step, py+dy*step))
            if len(points) > 4096:
                raise ValueError("CONTOUR_TOO_COMPLEX")
        if len(points) > config.CONTOUR_MAX_POINTS:
            points = [points[i*len(points)//config.CONTOUR_MAX_POINTS]
                      for i in range(config.CONTOUR_MAX_POINTS)]
        return {"ok": True, "reason": "OK", "boundary_points_px": points,
                "sample_step_px": step, "component_sample_count": len(largest),
                "search_bounds_raw_px":(x0,y0,x1,y1),"search_expansions":_expansions}
    except (ValueError, TypeError, IndexError, AttributeError, OverflowError) as exc:
        return {"ok": False, "reason": str(exc), "boundary_points_px": []}


def measure_bomb_from_boundary(calibration, boundary_points_px, color_id, frame_id,
                               geometry_confirmed=False, stable=False):
    """Pure measurement entry for real extracted raw-pixel silhouettes or offline tests."""
    if not _valid_color(color_id):
        return _error("INVALID_COLOR_ID", frame_id)
    if not geometry_confirmed:
        return _error("RAW_GEOMETRY_NOT_CONFIRMED", frame_id)
    if not stable:
        return _error("OBSERVATION_NOT_STABLE", frame_id)
    result = fit_sphere_center(calibration.camera, boundary_points_px, config.BOMB_DIAMETER_MM,
                              config.SPHERE_MAX_ERROR_PX, config.SPHERE_MIN_INLIER_RATIO,
                              config.SPHERE_MIN_COVERAGE_DEG, config.SPHERE_MIN_RADIUS_PX,
                              config.SPHERE_MAX_DISTANCE_MM)
    if not result["ok"]:
        return _error(result["reason"], frame_id)
    transformed = calibration.point_to_tool(result["position_camera_mm"], config.REPORT_CANDIDATE_TRANSFORMS)
    result.update(frame_id=frame_id, color_id=color_id, color=config.COLOR_NAMES[color_id],
                  geometry_valid=True, geometry_id=config.GEOMETRY_ID,
                  position_tool_mm=transformed["position_tool_mm"],
                  handeye_accepted=calibration.handeye_accepted,
                  control_ready=transformed["control_ready"],
                  coordinate_frame="E_commanded_gripper_center", units="mm")
    result["ok"], result["reason"] = transformed["ok"], transformed["reason"]
    return result


def measure_bombs(calibration, raw, detections, frame_id, geometry_confirmed=False,
                  stable=False, source="external_detections"):
    """Measure all unambiguous colored spheres; convert image color only once.

    detections must already be in full raw image coordinates. This function
    also allows real detections from another adapter; their source is explicit.
    """
    if not geometry_confirmed or (raw.width(), raw.height()) != config.IMAGE_SIZE:
        return dict(_error("RAW_GEOMETRY_NOT_CONFIRMED", frame_id), observations=[])
    if not stable:
        return dict(_error("OBSERVATION_NOT_STABLE", frame_id), observations=[])
    if not detections:
        return dict(_error("TARGET_BOMB_NOT_FOUND", frame_id), observations=[])
    if len(detections) > config.MAX_DETECTIONS:
        return dict(_error("TOO_MANY_DETECTIONS", frame_id), observations=[])
    observations = []
    rgb = raw.to_rgb565()
    try:
        for detection in detections:
            color = detection["color_id"]
            if not _valid_color(color):
                result = _error("INVALID_COLOR_ID", frame_id)
            elif sum(d["color_id"] == color for d in detections) != 1:
                result = _error("MULTIPLE_SAME_COLOR_TARGETS", frame_id)
            elif boundary_checks() and detection.get("bbox_clipped", False) and not config.BOMB_FULL_FRAME_SEARCH:
                result = _error("DETECTION_CLIPPED", frame_id)
            else:
                contour = extract_ball_boundary(rgb, detection["bbox_raw_px"], color)
                if not contour["ok"]:
                    result = _error(contour["reason"], frame_id)
                else:
                    result = measure_bomb_from_boundary(calibration, contour["boundary_points_px"],
                                                        color, frame_id, geometry_confirmed, stable)
                    result["contour_sample_step_px"] = contour["sample_step_px"]
            result.update(color_id=color, detection_score=detection.get("score"), source=source)
            if source != "yolo":
                result["control_ready"] = False
            observations.append(result)
    finally:
        rgb = None
    return {"ok": all(o["ok"] for o in observations),
            "reason": "OK" if all(o["ok"] for o in observations) else "ONE_OR_MORE_MEASUREMENTS_INVALID",
            "frame_id": frame_id, "observations": observations,
            "control_ready": all(o["control_ready"] for o in observations)}


class BombScanSession:
    """A controlled pass, from before slot 1 to after slot 3.

    Each color exists once. Direction and the starting boundary are caller
    declarations, NOT inferred from arbitrary detection-list order or time.
    The configured gate must expose at most one sphere at a time for order counting.
    """
    def __init__(self, target_color_id, scan_order_confirmed=False, start_boundary_confirmed=False,
                 gate=None, min_hits=None, release_frames=None):
        if not _valid_color(target_color_id):
            raise ValueError("INVALID_COLOR_ID")
        self.target = target_color_id
        self.confirmed = scan_order_confirmed is True and start_boundary_confirmed is True
        self.gate = tuple(gate or config.SCAN_GATE_RAW_PX)
        self.min_hits = config.SCAN_MIN_HITS if min_hits is None else min_hits
        self.release_frames = config.SCAN_RELEASE_FRAMES if release_frames is None else release_frames
        if len(self.gate) != 4 or not all(finite(v) for v in self.gate) or min(self.gate[2:]) <= 0:
            raise ValueError("INVALID_SCAN_GATE")
        if type(self.min_hits) is not int or self.min_hits < 2 or type(self.release_frames) is not int or self.release_frames < 1:
            raise ValueError("INVALID_SCAN_STABILITY")
        self.order, self.candidate, self.hits, self.latched = [], None, 0, None
        self.center, self.blank_frames, self.last_frame = None, 0, -1
        self.failed, self.closed, self.completed = False, False, False
        self.require_release=True  # Runtime chooses ordinal scanning explicitly.
        self.ambiguous_frames=0
        self.ambiguous_limit=3
        self.failure_reason=None
        self.last_detector_error=None

    def drop_frame(self,reason):
        """An unusable frame breaks hits, not previously confirmed identities."""
        self.candidate,self.hits,self.center=None,0,None
        self.blank_frames=self.ambiguous_frames=0
        self.last_detector_error=str(reason)

    def fail(self,reason):
        self.failed=True
        self.failure_reason=str(reason)
        return self._state(reason)

    def _state(self, reason, complete=False):
        mapping = {color: i+1 for i, color in enumerate(self.order)} if complete else None
        return {"ok": complete, "reason": reason, "complete": complete,
                "observed_order": list(self.order), "color_to_slot": mapping,
                "target_slot": mapping[self.target] if mapping is not None else None}

    def update(self, detections, frame_id):
        if self.completed:
            return self._state("SCAN_ALREADY_COMPLETE", complete=True)
        if self.closed:
            return self._state("SCAN_CLOSED")
        if not self.confirmed:
            return self._state("SCAN_ORDER_NOT_CONFIRMED")
        if self.failed:
            return self._state(self.failure_reason or "SCAN_ORDER_AMBIGUOUS_RESTART_REQUIRED")
        if type(frame_id) is not int or frame_id <= self.last_frame:
            return self._state("STALE_OR_DUPLICATE_FRAME")
        self.last_frame = frame_id
        x, y, w, h = self.gate
        eligible = []
        for det in detections:
            box = det["bbox_raw_px"]
            cx, cy = box[0]+box[2]*0.5, box[1]+box[3]*0.5
            if x <= cx < x+w and y <= cy < y+h:
                if not _valid_color(det["color_id"]):
                    return self._state("INVALID_COLOR_ID")
                eligible.append((det["color_id"], cx, cy))
        if not eligible:
            self.candidate, self.hits, self.center = None, 0, None
            self.blank_frames += 1
            if self.blank_frames >= self.release_frames:
                self.latched = None
            return self._state("SCAN_WAIT_TARGET")
        self.blank_frames = 0
        if len(eligible) != 1:
            self.candidate, self.hits, self.center = None, 0, None
            self.ambiguous_frames+=1
            if self.require_release or self.ambiguous_frames>=self.ambiguous_limit:
                return self.fail("SCAN_MULTIPLE_TARGETS_IN_GATE")
            return self._state("SCAN_WAIT_SINGLE_TARGET")
        self.ambiguous_frames=0
        color, cx, cy = eligible[0]
        if not self.require_release:
            if color in self.order:
                self.candidate,self.hits,self.center=None,0,None
                return self._state("SCAN_ALREADY_COUNTED")
            self.latched=None
        if self.latched is not None:
            if color != self.latched:
                return self.fail("SCAN_TRANSITION_WITHOUT_RELEASE")
            return self._state("SCAN_ALREADY_COUNTED")
        if color in self.order:
            return self.fail("SCAN_REVISIT_ORDER_AMBIGUOUS")
        close = self.center is not None and (cx-self.center[0])**2+(cy-self.center[1])**2 <= config.SCAN_MAX_STEP_PX**2
        self.hits = self.hits+1 if self.candidate == color and close else 1
        self.candidate, self.center = color, (cx, cy)
        if self.hits >= self.min_hits:
            self.order.append(color)
            self.latched = color
            self.candidate, self.hits = None, 0
            if len(self.order) == 3:
                self.completed = True
                return self._state("OK", complete=True)
            return self._state("SCAN_TARGET_RECORDED")
        return self._state("SCAN_STABILIZING")

    def finish(self):
        # Three confirmed unique colors complete automatically: no end command.
        if self.completed:
            return self._state("OK", complete=True)
        if self.closed:
            return self._state("SCAN_CLOSED")
        self.closed = True
        if not self.confirmed:
            return self._state("SCAN_ORDER_NOT_CONFIRMED")
        if self.failed:
            return self._state(self.failure_reason or "SCAN_ORDER_AMBIGUOUS_RESTART_REQUIRED")
        if len(self.order) != 3 or set(self.order) != {1, 2, 3}:
            return self._state("SCAN_INCOMPLETE")
        return self._state("OK", complete=True)


class BombTask:
    def __init__(self, calibration, detector):
        self.calibration, self.detector = calibration, detector
        self.mode, self.scan = "IDLE", None
        self.target_color, self.slot_id, self.last_frame = None, None, -1

    def start_scan(self, target_color_id, scan_order_confirmed=False,
                   start_boundary_confirmed=False, gate=None):
        self.scan = BombScanSession(target_color_id, scan_order_confirmed, start_boundary_confirmed, gate)
        self.mode, self.last_frame = "SCAN", -1

    def finish_scan(self):
        if self.mode not in ("SCAN", "DONE") or self.scan is None:
            return _error("SCAN_NOT_ACTIVE")
        self.mode = "DONE"
        return self.scan.finish()

    def start_refine(self, color_id, slot_id=None):
        if not _valid_color(color_id) or (slot_id is not None and not _valid_color(slot_id)):
            raise ValueError("INVALID_TARGET")
        self.target_color, self.slot_id = color_id, slot_id
        self.mode, self.last_frame = "REFINE", -1

    def poll(self, raw, frame_id, geometry_confirmed=False, stable=False):
        if self.mode not in ("SCAN", "REFINE"):
            return _error("TASK_NOT_ACTIVE", frame_id)
        if type(frame_id) is not int or frame_id <= self.last_frame:
            return _error("STALE_OR_DUPLICATE_FRAME", frame_id)
        self.last_frame = frame_id
        if not geometry_confirmed or (raw.width(), raw.height()) != config.IMAGE_SIZE:
            return _error("RAW_GEOMETRY_NOT_CONFIRMED", frame_id)
        if self.mode == "REFINE" and not stable:
            return _error("OBSERVATION_NOT_STABLE", frame_id)
        detection = self.detector.detect(raw)
        if not detection["ok"]:
            # Retry a transient detector error without losing confirmed order.
            if self.mode == "SCAN":self.scan.drop_frame(detection["reason"])
            return _error(detection["reason"], frame_id)
        if self.mode == "SCAN":
            result = self.scan.update(detection["detections"], frame_id)
            result["frame_id"] = frame_id
            if result["complete"]:
                self.mode = "DONE"
            return result
        matches = [d for d in detection["detections"] if d["color_id"] == self.target_color]
        if not matches:
            return _error("TARGET_BOMB_NOT_FOUND", frame_id)
        rx,ry,rw,rh=config.BOMB_DETECTION_ROI_RAW_PX
        matches=[d for d in matches if
                 rx<=d["bbox_raw_px"][0]+d["bbox_raw_px"][2]*.5<rx+rw and
                 ry<=d["bbox_raw_px"][1]+d["bbox_raw_px"][3]*.5<ry+rh]
        if not matches:
            return _error("BOMB_TARGET_OUTSIDE_DETECTION_ROI",frame_id)
        if len(matches) != 1:
            return _error("MULTIPLE_SAME_COLOR_TARGETS", frame_id)
        if boundary_checks() and matches[0].get("bbox_clipped", False) and not config.BOMB_FULL_FRAME_SEARCH:
            return _error("DETECTION_CLIPPED", frame_id)
        measured = measure_bombs(self.calibration, raw, matches, frame_id, geometry_confirmed,
                                 stable, detection.get("source", "unknown"))
        result = measured["observations"][0]
        result["slot_id"] = self.slot_id
        return result

    def cancel(self):
        self.mode, self.scan = "CANCELLED", None

    def close(self):
        self.cancel()
        self.detector.close()
