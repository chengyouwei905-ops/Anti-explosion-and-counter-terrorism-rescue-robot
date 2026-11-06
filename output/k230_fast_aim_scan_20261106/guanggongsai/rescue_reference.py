from boundary_policy import enabled as boundary_checks
"""Measured rescue-table layout and fresh native rectangle observations.

No table pose is cached across frames; ROI and physical orientation are explicit.
"""
import rescue_config as settings
from config import IMAGE_SIZE
from geometry import finite, transform_point
from rescue_geometry import RescueTablePoseEstimator


class RescueLayout:
    def __init__(self, data):
        self.data = dict(data)

    def ready_reason(self):
        d = self.data
        w,h = d.get("table_width_mm"),d.get("table_depth_mm")
        if not finite(w) or not finite(h) or not 0 < h < w:
            return "INVALID_TABLE_DIMENSIONS"
        if d.get("dimensions_confirmed") is not True:
            return "TABLE_DIMENSIONS_NOT_CONFIRMED"
        if d.get("slot_layout_confirmed") is not True or d.get("slot_centers_table_mm") is None:
            return "SLOT_LAYOUT_NOT_CONFIRMED"
        points = d["slot_centers_table_mm"]
        if not isinstance(points,(list,tuple)) or len(points) != 3:
            return "INVALID_SLOT_LAYOUT"
        for p in points:
            if not isinstance(p,(list,tuple)) or len(p) != 3 or not all(finite(v) for v in p):
                return "INVALID_SLOT_LAYOUT"
            if abs(p[2]) > 1e-5 or abs(p[0]) > w/2-25 or abs(p[1]) > h/2-25:
                return "SLOT_CENTER_OUTSIDE_TABLE"
        if not points[0][0] < points[1][0] < points[2][0]:
            return "SLOT_AXIS_ORDER_INVALID"
        if d.get("slot_axis_matches_scan_order_confirmed") is not True:
            return "SLOT_AXIS_NOT_CONFIRMED"
        return "OK"

    def native_ready_reason(self,has_corner_adapter=False):
        reason = self.ready_reason()
        if reason != "OK": return reason
        roi = (0,0,IMAGE_SIZE[0],IMAGE_SIZE[1]) if settings.TABLE_FULL_FRAME_SEARCH else self.data.get("table_top_roi_raw_px")
        if not isinstance(roi,(list,tuple)) or len(roi) != 4 or not all(type(v) is int for v in roi):
            return "TABLE_TOP_ROI_NOT_CONFIGURED"
        x,y,w,h = roi
        if min(x,y) < 0 or min(w,h) <= 0 or x+w > IMAGE_SIZE[0] or y+h > IMAGE_SIZE[1]:
            return "INVALID_TABLE_TOP_ROI"
        if not has_corner_adapter and self.data.get("top_view_image_corner_order_confirmed") is not True:
            return "TABLE_CORNER_ORDER_NOT_CONFIRMED"
        return "OK"

    def slot_point(self,slot_id):
        reason = self.ready_reason()
        if reason != "OK": raise ValueError(reason)
        if type(slot_id) is not int or not 1 <= slot_id <= 3:
            raise ValueError("INVALID_SLOT_ID")
        return list(self.data["slot_centers_table_mm"][slot_id-1])

    def grasp_point(self,slot_id):
        point = self.slot_point(slot_id)
        height = self.data.get("grasp_height_above_table_mm")
        if self.data.get("grasp_height_confirmed") is not True or height is None:
            return None
        if not finite(height) or not 0 <= height <= 200:
            raise ValueError("INVALID_GRASP_HEIGHT")
        point[2] = height
        return point


def order_restricted_top_view(corners):
    """Only for a physically confirmed view: far edge upper, +P.X to image right.

    Image sorting alone cannot label an unmarked table. The caller must confirm
    this restricted-view convention. Strongly rotated views are rejected.
    """
    if len(corners) != 4 or any(len(p) != 2 or not all(finite(v) for v in p) for p in corners):
        raise ValueError("INVALID_TABLE_CORNERS")
    ordered = sorted(corners,key=lambda p:p[1])
    top,bottom = sorted(ordered[:2],key=lambda p:p[0]),sorted(ordered[2:],key=lambda p:p[0])
    tl,tr,bl,br = top[0],top[1],bottom[0],bottom[1]
    for left,right in ((tl,tr),(bl,br)):
        if right[0]-left[0] <= 2*abs(right[1]-left[1]):
            raise ValueError("TABLE_VIEW_ORIENTATION_UNSUPPORTED")
    if min(bl[1]-tl[1],br[1]-tr[1]) <= 5:
        raise ValueError("TABLE_VIEW_ORIENTATION_UNSUPPORTED")
    return [tl,tr,br,bl]


class NativeRescueTableReference:
    """find_rects over the whole original frame; ROI is a candidate filter only.

    Configure a ROI identifying the top face, not a side face or arbitrary
    black rectangle. A custom corner_adapter(gray, rect) may supply physical
    corner order instead of the explicitly confirmed restricted-view convention.
    """
    def __init__(self,calibration,layout,corner_adapter=None):
        if corner_adapter is not None and not callable(corner_adapter):
            raise ValueError("INVALID_TABLE_CORNER_ADAPTER")
        self.calibration,self.layout,self.corner_adapter = calibration,layout,corner_adapter
        self.estimator = None
        self.closed,self.last_frame = False,-1

    def readiness(self):
        if self.closed: return "TABLE_REFERENCE_CLOSED"
        return self.layout.native_ready_reason(self.corner_adapter is not None)

    def _failure(self,reason,frame_id):
        return {"ok":False,"reason":reason,"frame_id":frame_id,"T_camera_from_table":None}

    def _black_fraction(self,gray,pose):
        w,h = self.layout.data["table_width_mm"],self.layout.data["table_depth_mm"]
        black,total = 0,0
        for x in (-.4,-.2,0,.2,.4):
            for y in (-.3,.3):
                point = transform_point(pose["T_camera_from_table"],[x*w,y*h,0])
                u,v = self.calibration.camera.project_ray(point)
                u,v = int(round(u)),int(round(v))
                if not 0 <= u < IMAGE_SIZE[0] or not 0 <= v < IMAGE_SIZE[1]:
                    if boundary_checks():raise ValueError("TABLE_SAMPLE_OUTSIDE_IMAGE")
                    continue
                value = gray.get_pixel(u,v)
                if not finite(value): raise ValueError("GRAYSCALE_PIXEL_API_UNSUPPORTED")
                black += int(value <= settings.TABLE_BLACK_MAX_GRAY)
                total += 1
        if not total:raise ValueError("TABLE_NO_VISIBLE_SAMPLES")
        return black/float(total)

    def observe(self,raw,frame_id,geometry_confirmed=False):
        reason = self.readiness()
        if reason != "OK": return self._failure(reason,frame_id)
        if type(frame_id) is not int or frame_id <= self.last_frame:
            return self._failure("STALE_OR_DUPLICATE_TABLE_FRAME",frame_id)
        self.last_frame = frame_id
        if not geometry_confirmed or (raw.width(),raw.height()) != IMAGE_SIZE:
            return self._failure("RAW_GEOMETRY_NOT_CONFIRMED",frame_id)
        gray = None
        try:
            if self.estimator is None:
                self.estimator = RescueTablePoseEstimator(self.calibration.camera,
                                    self.layout.data["table_width_mm"],self.layout.data["table_depth_mm"])
            gray = raw.to_grayscale()
            # Do not crop or pass ROI: keep the corner coordinates unambiguously raw.
            rectangles = gray.find_rects(threshold=settings.RECTANGLE_THRESHOLD)
            if len(rectangles) > settings.MAX_TABLE_CANDIDATES:
                return self._failure("TOO_MANY_TABLE_CANDIDATES",frame_id)
            x,y,w,h = ((0,0,IMAGE_SIZE[0],IMAGE_SIZE[1]) if settings.TABLE_FULL_FRAME_SEARCH
                       else self.layout.data["table_top_roi_raw_px"])
            accepted,reasons = [],[]
            for rectangle in rectangles:
                corners = rectangle.corners()
                if len(corners) != 4 or not all(x <= p[0] < x+w and y <= p[1] < y+h for p in corners):
                    continue
                try:
                    if self.corner_adapter is None:
                        corners = order_restricted_top_view(corners)
                    else:
                        corners = self.corner_adapter(gray,rectangle)
                        if corners is None: continue
                    pose = self.estimator.estimate_table(corners,True,True)
                    if not pose["ok"]:
                        reasons.append(pose["reason"]); continue
                    fraction = self._black_fraction(gray,pose)
                    if fraction < settings.TABLE_MIN_BLACK_FRACTION:
                        reasons.append("TABLE_TOP_COLOR_INCONSISTENT"); continue
                    pose.update(frame_id=frame_id,source="native_table_rectangle")
                    pose["quality"]["black_sample_fraction"] = fraction
                    accepted.append(pose)
                except (ValueError,TypeError,IndexError) as exc:
                    reasons.append(str(exc))
            if len(accepted) > 1:
                return self._failure("TABLE_REFERENCE_AMBIGUOUS",frame_id)
            if not accepted:
                return self._failure(reasons[0] if reasons else "TABLE_TOP_NOT_FOUND",frame_id)
            return accepted[0]
        except Exception as exc:
            return self._failure("TABLE_REFERENCE_ERROR:"+str(exc),frame_id)
        finally:
            gray = None

    def close(self):
        self.closed,self.estimator = True,None
