"""CanMV color target detector and two annulus boundaries. No hardware ownership."""
import math
from boundary_policy import enabled as boundary_checks,visible_edges
import aim_config as settings
from config import IMAGE_SIZE, COLOR_NAMES
from geometry import finite
from lab_color import matches_lab,rgb_to_lab_reference,native_lab_mask


def valid_color(color):
    return type(color) is int and 1 <= color <= 3


def _color_matches(rgb, color):
    # Host fallback; hardware uses the same native LAB binary as find_blobs.
    return matches_lab(rgb_to_lab_reference(rgb),settings.LAB_THRESHOLDS[color-1])


def _neighbors(i, columns, rows):
    x, y = i % columns, i // columns
    if x: yield i-1
    if x+1 < columns: yield i+1
    if y: yield i-columns
    if y+1 < rows: yield i+columns


def _component(mask, start, columns, rows, old, new):
    mask[start] = new
    queue, head = [start], 0
    while head < len(queue):
        current = queue[head]
        head += 1
        for adjacent in _neighbors(current, columns, rows):
            if mask[adjacent] == old:
                mask[adjacent] = new
                queue.append(adjacent)
    return queue


def extract_target_rings(rgb, bbox_raw_px, color_id):
    """Extract outside and largest enclosed non-color hole of one colored annulus.

    Interior black score rings remain inside the same non-color hole. Open,
    clipped annuli, small highlights and multiple large components are refused.
    """
    try:
        if not valid_color(color_id):
            raise ValueError("INVALID_COLOR_ID")
        if (rgb.width(), rgb.height()) != IMAGE_SIZE:
            raise ValueError("IMAGE_SIZE_MISMATCH")
        if len(bbox_raw_px) != 4 or not all(finite(v) for v in bbox_raw_px) or min(bbox_raw_px[2:]) <= 0:
            raise ValueError("INVALID_TARGET_ROI")
        x, y, width, height = bbox_raw_px
        margin = max(5, int(max(width, height)*0.12))
        x0, y0 = max(0, int(math.floor(x))-margin), max(0, int(math.floor(y))-margin)
        x1, y1 = min(IMAGE_SIZE[0], int(math.ceil(x+width))+margin), min(IMAGE_SIZE[1], int(math.ceil(y+height))+margin)
        if x1 <= x0 or y1 <= y0:
            raise ValueError("INVALID_TARGET_ROI")
        step = 1
        while ((x1-x0+step-1)//step)*((y1-y0+step-1)//step) > settings.MAX_ROI_CELLS:
            step += 1
        columns, rows = (x1-x0+step-1)//step, (y1-y0+step-1)//step
        lab_mask=native_lab_mask(rgb,settings.LAB_THRESHOLDS[color_id-1])
        mask = bytearray(columns*rows)
        for gy in range(rows):
            for gx in range(columns):
                pixel = (lab_mask if lab_mask is not None else rgb).get_pixel(
                    x0+gx*step, y0+gy*step, rgbtuple=True)
                if pixel is None or len(pixel) != 3:
                    raise ValueError("RGB_PIXEL_API_UNSUPPORTED")
                mask[gy*columns+gx] = int(any(pixel) if lab_mask is not None else _color_matches(pixel,color_id))
        largest, second = [], 0
        for i in range(len(mask)):
            if mask[i] == 1:
                component = _component(mask, i, columns, rows, 1, 2)
                if len(component) > len(largest):
                    second, largest = max(second, len(largest)), component
                else:
                    second = max(second, len(component))
        if len(largest) < settings.MIN_COMPONENT_SAMPLES:
            raise ValueError("TARGET_COLOR_TOO_SMALL")
        if second > len(largest)*0.3:
            raise ValueError("TARGET_COLOR_COMPONENTS_AMBIGUOUS")
        if boundary_checks() and any(i % columns in (0, columns-1) or i // columns in (0, rows-1) for i in largest):
            raise ValueError("TARGET_CONTOUR_CLIPPED")
        topology = bytearray(columns*rows)
        for i in largest: topology[i] = 1
        # Mark background connected to any ROI border as exterior=2.
        for i in range(len(topology)):
            if topology[i] == 0 and (i % columns in (0, columns-1) or i // columns in (0, rows-1)):
                _component(topology, i, columns, rows, 0, 2)
        hole, other_hole = [], 0
        for i in range(len(topology)):
            if topology[i] == 0:
                component = _component(topology, i, columns, rows, 0, 3)
                if len(component) > len(hole):
                    other_hole, hole = max(other_hole, len(hole)), component
                else:
                    other_hole = max(other_hole, len(component))
        if boundary_checks() and len(hole) < settings.MIN_HOLE_SAMPLES:
            raise ValueError("TARGET_RING_OPEN_OR_NO_HOLE")
        if boundary_checks() and other_hole > len(hole)*0.2:
            raise ValueError("TARGET_INNER_RING_AMBIGUOUS")
        ratio = len(hole)/float(len(hole)+len(largest))
        if boundary_checks() and not settings.HOLE_AREA_RATIO[0] <= ratio <= settings.HOLE_AREA_RATIO[1]:
            raise ValueError("TARGET_HOLE_SIZE_INCONSISTENT")
        # Other small holes (highlights) do not belong to the physical 55mm rim.
        for i in range(len(topology)):
            if topology[i] == 3: topology[i] = 0
        for i in hole: topology[i] = 3
        outer, inner = [], []
        # When a ring is cut/open, its inner non-colour region is connected
        # to the exterior. Classify observed edge normals around the coarse
        # center instead of inventing a closed boundary along the image cut.
        visible_mode=(not boundary_checks() and (len(hole)<settings.MIN_HOLE_SAMPLES or
                       any(i%columns in (0,columns-1) or i//columns in (0,rows-1) for i in largest)))
        center_x,center_y=x+width*.5,y+height*.5
        for i in largest:
            gx, gy = i % columns, i // columns
            px, py = x0+gx*step, y0+gy*step
            for adjacent,dx,dy in visible_edges(i,columns,rows):
                if topology[adjacent]==1:continue
                edge=(px+dx*step,py+dy*step)
                if visible_mode:
                    radial=dx*(edge[0]-center_x)+dy*(edge[1]-center_y)
                    if radial>0:outer.append(edge)
                    elif radial<0:inner.append(edge)
                elif topology[adjacent]==2:outer.append(edge)
                elif topology[adjacent]==3:inner.append(edge)
            if len(outer)+len(inner) > 4096:
                raise ValueError("TARGET_CONTOUR_TOO_COMPLEX")
        def capped(points):
            if len(points) > settings.MAX_POINTS_PER_RING:
                return [points[i*len(points)//settings.MAX_POINTS_PER_RING] for i in range(settings.MAX_POINTS_PER_RING)]
            return points
        outer, inner = capped(outer), capped(inner)
        if len(outer) < 24 or len(inner) < 20:
            raise ValueError("TARGET_RINGS_TOO_SMALL")
        return {"ok": True, "reason": "OK", "outer_points_px": outer,
                "inner_points_px": inner, "sample_step_px": step, "hole_area_ratio": ratio,
                "extraction_mode":"visible_edges" if visible_mode else "closed_ring",
                "boundary_checks_enabled":boundary_checks()}
    except (ValueError, TypeError, IndexError, AttributeError, OverflowError) as exc:
        return {"ok": False, "reason": str(exc), "outer_points_px": [], "inner_points_px": []}


class ColorTargetDetector:
    """ROI-only LAB scan, or target-only LAB contour extraction for measurement."""
    def __init__(self,roi=None,coarse_only=False,target_color=None):
        self.roi=tuple(roi) if roi is not None else None
        if self.roi is not None:
            if (len(self.roi)!=4 or any(type(v) is not int for v in self.roi) or
                    min(self.roi[:2])<0 or min(self.roi[2:])<=0 or
                    self.roi[0]+self.roi[2]>IMAGE_SIZE[0] or
                    self.roi[1]+self.roi[3]>IMAGE_SIZE[1]):
                raise ValueError("INVALID_TARGET_ROI")
        self.coarse_only=coarse_only
        self.set_target_color(target_color)
        self.ready = True

    def set_target_color(self,color):
        if color is not None and not valid_color(color):raise ValueError("INVALID_COLOR_ID")
        self.target_color=color

    def load(self):
        self.ready = True
        return {"ok": True, "reason": "OK"}

    def detect(self, raw):
        if not self.ready:
            return {"ok": False, "reason": "TARGET_DETECTOR_CLOSED", "detections": []}
        rgb = None
        try:
            if (raw.width(), raw.height()) != IMAGE_SIZE:
                raise ValueError("IMAGE_SIZE_MISMATCH")
            rgb = raw.to_rgb565()
            thresholds=(settings.LAB_THRESHOLDS if self.target_color is None else
                        (settings.LAB_THRESHOLDS[self.target_color-1],))
            options={"pixels_threshold":getattr(settings,"TARGET_MIN_COLOR_PIXELS",200),
                     "area_threshold":getattr(settings,"TARGET_MIN_BBOX_AREA_PX",900),
                     "merge":False,"x_stride":2,"y_stride":2}
            if self.roi is not None:options["roi"]=self.roi
            blobs=rgb.find_blobs(thresholds,**options)
            candidates,detections,rejected=[],[],[]
            for blob in blobs:
                code = blob.code()
                if code not in (1,2,4) or (self.target_color is not None and code!=1):
                    rejected.append({"color_id": None, "reason": "TARGET_COLOR_AMBIGUOUS"})
                    continue
                color=self.target_color if self.target_color is not None else {1:1,2:2,4:3}[code]
                box = tuple(blob.rect())
                # Filter before contour work and candidate-count rejection.
                x,y,w,h=self.roi if self.roi is not None else settings.TARGET_DETECTION_ROI_RAW_PX
                if not (x<=box[0]+box[2]*.5<x+w and y<=box[1]+box[3]*.5<y+h):continue
                candidates.append((color,box))
            if len(candidates)>settings.MAX_CANDIDATES:
                raise ValueError("TOO_MANY_TARGET_CANDIDATES")
            for color,box in candidates:
                if self.coarse_only:
                    detections.append({"color_id":color,"label":COLOR_NAMES[color]+"_target",
                                       "bbox_raw_px":box,"score":1.0,"score_kind":"lab_blob_only"})
                    continue
                contour = extract_target_rings(rgb, box, color)
                if not contour["ok"]:
                    rejected.append({"color_id": color, "reason": contour["reason"]})
                    continue
                detections.append({"color_id": color, "label": COLOR_NAMES[color]+"_target",
                                   "bbox_raw_px": box, "score": 1.0,
                                   "score_kind": "annulus_topology_pass", "contours": contour})
            return {"ok": True, "reason": "OK", "source": "color_target",
                    "detections": detections, "rejected": rejected}
        except Exception as exc:
            return {"ok": False, "reason": "TARGET_DETECTOR_ERROR:"+str(exc), "detections": []}
        finally:
            rgb = None

    def close(self):
        self.ready = False
