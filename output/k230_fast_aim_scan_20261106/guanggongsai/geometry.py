"""Pure math for raw pixels, spheres and E_from_C transforms. mm, radians."""
import math
from boundary_policy import enabled as boundary_checks


def finite(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and x == x and abs(x) < 1e30


def dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    length = norm(a)
    if length < 1e-12 or not finite(length):
        raise ValueError("DEGENERATE_VECTOR")
    return [x / length for x in a]


def validate_rigid(T):
    if len(T) != 4 or any(len(row) != 4 for row in T):
        raise ValueError("BAD_TRANSFORM_SHAPE")
    if not all(finite(x) for row in T for x in row):
        raise ValueError("NONFINITE_TRANSFORM")
    if max(abs(T[3][i] - (1.0 if i == 3 else 0.0)) for i in range(4)) > 1e-5:
        raise ValueError("BAD_HOMOGENEOUS_ROW")
    cols = [[T[i][j] for i in range(3)] for j in range(3)]
    if max(abs(dot(cols[i], cols[j]) - (1.0 if i == j else 0.0))
           for i in range(3) for j in range(3)) > 0.002 or dot(cols[0], cross(cols[1], cols[2])) < 0.998:
        raise ValueError("NOT_A_PROPER_ROTATION")
    return T


def compose(A, B):
    validate_rigid(A)
    validate_rigid(B)
    return [[sum(A[i][k]*B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def transform_point(T, point):
    validate_rigid(T)
    if len(point) != 3 or not all(finite(x) for x in point):
        raise ValueError("BAD_POINT")
    return [sum(T[i][j]*point[j] for j in range(3)) + T[i][3] for i in range(3)]


class CameraModel:
    def __init__(self, K, D, image_size=(640, 480)):
        if len(K) != 3 or any(len(row) != 3 for row in K) or len(D) != 5:
            raise ValueError("BAD_CAMERA_PARAMETERS")
        if not all(finite(v) for row in K for v in row) or not all(finite(v) for v in D):
            raise ValueError("BAD_CAMERA_PARAMETERS")
        if K[0][0] <= 0 or K[1][1] <= 0 or any(abs(v) > 1e-8 for v in
                (K[0][1], K[1][0], K[2][0], K[2][1], K[2][2]-1)):
            raise ValueError("UNSUPPORTED_CAMERA_MATRIX")
        self.K, self.D, self.image_size = K, D, tuple(image_size)
        self.fx, self.fy, self.cx, self.cy = K[0][0], K[1][1], K[0][2], K[1][2]

    def distort(self, x, y):
        k1, k2, p1, p2, k3 = self.D
        r2 = x*x+y*y
        radial = 1+k1*r2+k2*r2*r2+k3*r2*r2*r2
        return (x*radial+2*p1*x*y+p2*(r2+2*x*x),
                y*radial+p1*(r2+2*y*y)+2*p2*x*y)

    def ray(self, pixel):
        u, v = pixel
        if not finite(u) or not finite(v) or not (0 <= u < self.image_size[0] and 0 <= v < self.image_size[1]):
            raise ValueError("PIXEL_OUTSIDE_RAW_IMAGE")
        tx, ty = (u-self.cx)/self.fx, (v-self.cy)/self.fy
        x, y = tx, ty
        # Fixed-point Brown inversion with a verified forward residual.
        for _ in range(20):
            dx, dy = self.distort(x, y)
            ex, ey = tx-dx, ty-dy
            x, y = x+ex, y+ey
            if ex*ex+ey*ey < 1e-18:
                break
        dx, dy = self.distort(x, y)
        if (dx-tx)**2+(dy-ty)**2 > 1e-12:
            raise ValueError("UNDISTORT_DID_NOT_CONVERGE")
        return unit([x, y, 1.0])

    def project_ray(self, ray):
        if ray[2] <= 1e-9:
            raise ValueError("NONPOSITIVE_DEPTH")
        x, y = self.distort(ray[0]/ray[2], ray[1]/ray[2])
        return (self.fx*x+self.cx, self.fy*y+self.cy)


def _plane_from_three(a, b, c):
    n = unit(cross([b[i]-a[i] for i in range(3)], [c[i]-a[i] for i in range(3)]))
    k = dot(n, a)
    if k < 0:
        n, k = [-v for v in n], -k
    if not (0.01 < k < 1.0):
        raise ValueError("DEGENERATE_TANGENT_CONE")
    return n, math.sqrt(max(0.0, 1-k*k)), k


def _plane_fit(rays):
    # Smallest covariance eigenvector; Jacobi rotations need only standard math.
    mean = [sum(r[i] for r in rays)/len(rays) for i in range(3)]
    A = [[sum((r[i]-mean[i])*(r[j]-mean[j]) for r in rays) for j in range(3)] for i in range(3)]
    V = [[float(i == j) for j in range(3)] for i in range(3)]
    scale = max(abs(x) for row in A for x in row)
    if scale < 1e-14:
        raise ValueError("DEGENERATE_CONTOUR")
    A = [[x/scale for x in row] for row in A]
    for _ in range(24):
        p, q = max(((0, 1), (0, 2), (1, 2)), key=lambda ij: abs(A[ij[0]][ij[1]]))
        if abs(A[p][q]) < 1e-10:
            break
        angle = 0.5*math.atan2(2*A[p][q], A[q][q]-A[p][p])
        c, s = math.cos(angle), math.sin(angle)
        app, aqq, apq = A[p][p], A[q][q], A[p][q]
        A[p][p] = c*c*app-2*s*c*apq+s*s*aqq
        A[q][q] = s*s*app+2*s*c*apq+c*c*aqq
        A[p][q] = A[q][p] = 0.0
        for j in range(3):
            if j != p and j != q:
                ajp, ajq = A[j][p], A[j][q]
                A[j][p] = A[p][j] = c*ajp-s*ajq
                A[j][q] = A[q][j] = s*ajp+c*ajq
            vjp, vjq = V[j][p], V[j][q]
            V[j][p], V[j][q] = c*vjp-s*vjq, s*vjp+c*vjq
    idx = min(range(3), key=lambda i: A[i][i])
    n = unit([V[i][idx] for i in range(3)])
    if dot(n, mean) < 0:
        n = [-x for x in n]
    # Use cross products rather than 1-k^2 to retain float32 precision.
    sine = sum(norm(cross(n, r)) for r in rays)/len(rays)
    cosine = sum(dot(n, r) for r in rays)/len(rays)
    return n, sine, cosine


def _pixel_errors(camera, pixels, rays, axis, sine, cosine):
    errors = []
    for pixel, ray in zip(pixels, rays):
        parallel = dot(axis, ray)
        tangent = [ray[j]-parallel*axis[j] for j in range(3)]
        try:
            tangent = unit(tangent)
            ideal = [cosine*axis[j]+sine*tangent[j] for j in range(3)]
            u, v = camera.project_ray(ideal)
            errors.append(math.sqrt((u-pixel[0])**2+(v-pixel[1])**2))
        except ValueError:
            errors.append(1e9)
    return errors


def fit_sphere_center(camera, boundary_points_px, diameter_mm=40.0,
                      max_error_px=2.0, min_inlier_ratio=0.80,
                      min_coverage_deg=270.0, min_radius_px=8.0,
                      max_distance_mm=2000.0):
    """Fit the tangent cone of a known sphere, NOT a bbox diameter or ellipse center.

    Raw silhouette rays lie on a small circle on the unit sphere. Its axis
    points to the physical sphere center; distance = radius/sin(half angle).
    Orientation is unobservable. No frame history or hardware is accessed.
    """
    try:
        if not finite(diameter_mm) or diameter_mm <= 0:
            raise ValueError("INVALID_SPHERE_SIZE")
        points = list(boundary_points_px)
        if not 16 <= len(points) <= 512:
            raise ValueError("CONTOUR_POINT_COUNT")
        rays = [camera.ray(p) for p in points]
        count = len(rays)
        # Deterministic, bounded RANSAC. Points need not be angularly ordered.
        candidates = [_plane_fit(rays)]
        seed = 71237
        for _ in range(64):
            indices = []
            for _ in range(3):
                seed = (1664525*seed+1013904223) & 0xffffffff
                indices.append(seed % count)
            if len(set(indices)) != 3:
                continue
            try:
                candidates.append(_plane_from_three(*(rays[i] for i in indices)))
            except ValueError:
                pass
        best, best_score = None, None
        for axis, sine, cosine in candidates:
            if sine <= 1e-5 or axis[2] <= 0:
                continue
            errors = _pixel_errors(camera, points, rays, axis, sine, cosine)
            selected = [i for i, e in enumerate(errors) if e <= max_error_px]
            score = (len(selected), -sum(errors[i]**2 for i in selected))
            if best_score is None or score > best_score:
                best, best_score = selected, score
        strict=boundary_checks()
        required=max(16,int(math.ceil(count*min_inlier_ratio)))
        if strict and (best is None or len(best)<required):
            raise ValueError("SPHERE_CONTOUR_INCONSISTENT")
        if best is None or len(best)<3:
            best=list(range(count))
        for _ in range(2):
            axis, sine, cosine = _plane_fit([rays[i] for i in best])
            scale = math.sqrt(sine*sine+cosine*cosine)
            sine, cosine = sine/scale, cosine/scale
            errors = _pixel_errors(camera, points, rays, axis, sine, cosine)
            selected = [i for i, e in enumerate(errors) if e <= max_error_px]
            if strict and len(selected)<required:
                raise ValueError("SPHERE_CONTOUR_INCONSISTENT")
            if len(selected)>=3:best=selected
        if strict and sine*min(camera.fx, camera.fy) < min_radius_px:
            raise ValueError("SPHERE_TOO_SMALL")
        distance = diameter_mm*0.5/sine
        if not finite(distance) or not (diameter_mm*0.5 < distance <= max_distance_mm):
            raise ValueError("SPHERE_RANGE_INVALID")
        ex = unit(cross(axis, [0, 1, 0] if abs(axis[1]) < 0.9 else [1, 0, 0]))
        ey = cross(axis, ex)
        angles = sorted(math.atan2(dot(rays[i], ey), dot(rays[i], ex)) for i in best)
        gaps = [angles[i+1]-angles[i] for i in range(len(angles)-1)]
        gaps.append(angles[0]+2*math.pi-angles[-1])
        coverage = 360.0-max(gaps)*180.0/math.pi
        if strict and coverage < min_coverage_deg:
            raise ValueError("SPHERE_CONTOUR_INCOMPLETE")
        actual_inliers=sum(e<=max_error_px for e in errors)
        warnings=[]
        if actual_inliers<required:warnings.append("SPHERE_CONTOUR_INCONSISTENT")
        if coverage<min_coverage_deg:warnings.append("SPHERE_CONTOUR_INCOMPLETE")
        if sine*min(camera.fx,camera.fy)<min_radius_px:warnings.append("SPHERE_TOO_SMALL")
        center = [distance*v for v in axis]
        if center[2] <= diameter_mm*0.5:
            raise ValueError("NONPOSITIVE_DEPTH")
        return {"ok": True, "reason": "OK", "position_camera_mm": center,
                "reference_point": "sphere_center", "orientation_valid": False,
                "quality": {"contour_rms_px": math.sqrt(sum(errors[i]**2 for i in best)/len(best)),
                            "inlier_ratio": actual_inliers/float(count), "coverage_deg": coverage,
                            "boundary_checks_enabled":strict,"warnings":warnings,
                            "point_count": count, "diameter_mm": diameter_mm}}
    except (ValueError, TypeError, IndexError, ZeroDivisionError, OverflowError) as exc:
        return {"ok": False, "reason": str(exc), "position_camera_mm": None,
                "orientation_valid": False}
