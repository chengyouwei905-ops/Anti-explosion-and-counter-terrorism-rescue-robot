"""CanMV color target detector and two annulus boundaries. No hardware ownership."""
import math
from boundary_policy import enabled as boundary_checks,visible_edges
import aim_config as settings
from config import IMAGE_SIZE, COLOR_NAMES
from geometry import finite


def valid_color(color):
    return type(color) is int and 1 <= color <= 3


def _color_matches(rgb, color):
    r, g, b = [v/255.0 for v in rgb]
    high, low = max(r, g, b), min(r, g, b)
    delta = high-low
    if high < settings.MIN_VALUE or delta <= 0 or delta/high < settings.MIN_SATURATION:
        return False
    if high == r:
        hue = (60*(g-b)/delta) % 360.0
    elif high == g:
        hue = 60*(b-r)/delta+120
    else:
        hue = 60*(r-g)/delta+240
    return any(lo <= hue <= hi for lo, hi in settings.HUE_DEG[color])


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
        mask = bytearray(columns*rows)
        for gy in range(rows):
            for gx in range(columns):
                pixel = rgb.get_pixel(x0+gx*step, y0+gy*step, rgbtuple=True)
                if pixel is None or len(pixel) != 3:
                    raise ValueError("RGB_PIXEL_API_UNSUPPORTED")
                mask[gy*columns+gx] = int(_color_matches(pixel, color_id))
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
                "boundary_checks_enabled":boundary_checks(),
                "color_area_px":len(largest)*step*step}
    except (ValueError, TypeError, IndexError, AttributeError, OverflowError) as exc:
        return {"ok": False, "reason": str(exc), "outer_points_px": [], "inner_points_px": []}




def target_area_rejection(target, contours=None):
    """Minimum VGA bbox area and available annulus-color pixel count."""
    box=target.get("bbox_raw_px")
    if box is None or len(box)!=4 or not all(finite(v) for v in box) or min(box[2:])<=0:
        return "INVALID_TARGET_ROI"
    if box[2]*box[3] < settings.TARGET_MIN_BBOX_AREA_PX:
        return "TARGET_BBOX_TOO_SMALL"
    pixels=target.get("color_pixels")
    if pixels is None and contours is not None:pixels=contours.get("color_area_px")
    if pixels is not None and (not finite(pixels) or pixels<settings.TARGET_MIN_COLOR_PIXELS):
        return "TARGET_COLOR_TOO_SMALL"
    return None


def target_priority(target):
    """Larger bounding area first; equal areas prefer the working-region center."""
    x,y,w,h=target["bbox_raw_px"]
    rx,ry,rw,rh=settings.TARGET_DETECTION_ROI_RAW_PX
    distance=(x+w*.5-rx-rw*.5)**2+(y+h*.5-ry-rh*.5)**2
    return (-w*h,distance)


class ColorTargetDetector:
    """Native CanMV find_blobs for coarse color, explicit annulus topology check."""
    def __init__(self):
        self.ready = True

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
            blobs = rgb.find_blobs(settings.LAB_THRESHOLDS,
                                   pixels_threshold=settings.TARGET_MIN_COLOR_PIXELS,
                                   area_threshold=settings.TARGET_MIN_BBOX_AREA_PX,
                                   merge=False, x_stride=2, y_stride=2)
            candidates, detections, rejected = [], [], []
            for blob in blobs:
                code=blob.code()
                if code not in (1,2,4):
                    rejected.append({"color_id":None,"reason":"TARGET_COLOR_AMBIGUOUS"})
                    continue
                color={1:1,2:2,4:3}[code]
                candidate={"color_id":color,"bbox_raw_px":tuple(blob.rect())}
                # pixels() exists in the v1.8-75 image.blob API. Host adapters
                # without it are checked using contour color_area_px below.
                pixels=getattr(blob,"pixels",None)
                if pixels is not None:candidate["color_pixels"]=pixels()
                reason=target_area_rejection(candidate)
                if reason:
                    rejected.append({"color_id":color,"reason":reason})
                else:candidates.append(candidate)
            candidates.sort(key=target_priority)
            # Bound contour work instead of failing the whole frame on count.
            limited=len(candidates)>settings.MAX_CANDIDATES
            for candidate in candidates[:settings.MAX_CANDIDATES]:
                color,box=candidate["color_id"],candidate["bbox_raw_px"]
                contour=extract_target_rings(rgb,box,color)
                reason=contour.get("reason","TARGET_RINGS_TOO_SMALL") if not contour["ok"] else target_area_rejection(candidate,contour)
                if reason:
                    rejected.append({"color_id":color,"reason":reason})
                    continue
                candidate.update(label=COLOR_NAMES[color]+"_target",score=1.0,
                                 score_kind="annulus_topology_pass",contours=contour)
                detections.append(candidate)
            return {"ok":True,"reason":"OK","source":"color_target",
                    "detections":detections,"rejected":rejected,
                    "coarse_candidate_count":len(candidates),"candidates_limited":limited}
        except Exception as exc:
            return {"ok": False, "reason": "TARGET_DETECTOR_ERROR:"+str(exc), "detections": []}
        finally:
            rgb = None

    def close(self):
        self.ready = False
