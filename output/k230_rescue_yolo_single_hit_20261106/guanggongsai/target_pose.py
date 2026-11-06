"""Known concentric-circle center and plane fit, pure MicroPython math.

The image ellipse center is only an initialization, never the output center.
Full in-plane orientation is unobservable for an unmarked circular target.
"""
import math
from boundary_policy import enabled as boundary_checks
import aim_config as settings
from geometry import dot, cross, norm, unit, finite


def _solve(A, b):
    count = len(b)
    rows = []
    for i in range(count):
        scale = max(abs(v) for v in A[i])
        if not finite(scale) or scale < 1e-16:
            raise ValueError("TARGET_GEOMETRY_UNOBSERVABLE")
        rows.append([v/scale for v in A[i]]+[b[i]/scale])
    for i in range(count):
        pivot = max(range(i, count), key=lambda j: abs(rows[j][i]))
        if abs(rows[pivot][i]) < 1e-9:
            raise ValueError("TARGET_GEOMETRY_UNOBSERVABLE")
        rows[i], rows[pivot] = rows[pivot], rows[i]
        for j in range(i+1, count):
            factor = rows[j][i]/rows[i][i]
            for k in range(i, count+1):
                rows[j][k] -= factor*rows[i][k]
    result = [0.0]*count
    for i in range(count-1, -1, -1):
        result[i] = (rows[i][-1]-sum(rows[i][j]*result[j] for j in range(i+1, count)))/rows[i][i]
    if not all(finite(v) for v in result):
        raise ValueError("NONFINITE_TARGET_SOLUTION")
    return result


def _evaluate(parameters, samples, radius, focal, jacobian=False):
    C = [radius*p for p in parameters[:3]]
    a, b = parameters[3:]
    if not settings.MIN_DEPTH_MM <= C[2] <= settings.MAX_DEPTH_MM:
        raise ValueError("TARGET_DEPTH_OUT_OF_RANGE")
    N = [a, b, -1.0]
    plane_offset = dot(N, C)
    scale = focal/C[2]
    residuals, rows = [], []
    for ray, known_radius, _pixel in samples:
        denominator = dot(N, ray)
        if denominator >= -1e-6:
            raise ValueError("TARGET_PLANE_BEHIND_RAY")
        distance = plane_offset/denominator
        if distance <= 0:
            raise ValueError("TARGET_PLANE_BEHIND_CAMERA")
        q = [distance*ray[i]-C[i] for i in range(3)]
        length = norm(q)
        if length < 1e-6:
            raise ValueError("DEGENERATE_TARGET_POINT")
        error = (length-known_radius)*scale
        residuals.append(error)
        if jacobian:
            qray = dot(q, ray)
            row = [scale*radius*(qray*N[j]/denominator-q[j])/length for j in range(3)]
            row[2] -= error*radius/C[2]
            row += [scale*qray*(C[j]*denominator-plane_offset*ray[j])/(length*denominator*denominator)
                    for j in range(2)]
            rows.append(row)
    return residuals, rows


def _normal_equations(residuals, rows, robust=True):
    H = [[0.0]*5 for _ in range(5)]
    g = [0.0]*5
    cost = 0.0
    for error, row in zip(residuals, rows):
        limit = settings.MAX_REPROJECTION_PX
        weight = min(1.0, limit/max(abs(error), 1e-9)) if robust else 1.0
        cost += weight*error*error
        for i in range(5):
            g[i] += weight*row[i]*error
            for j in range(5):
                H[i][j] += weight*row[i]*row[j]
    return H, g, cost


def _cost(residuals):
    limit = settings.MAX_REPROJECTION_PX
    return sum(e*e if abs(e) <= limit else 2*limit*abs(e)-limit*limit for e in residuals)


def _refine(initial, samples, radius, focal):
    p = list(initial)
    damping = 0.001
    for _ in range(32):
        residuals, rows = _evaluate(p, samples, radius, focal, True)
        H, gradient, _ = _normal_equations(residuals, rows)
        old_cost = _cost(residuals)
        improved = False
        for _ in range(7):
            A = [[H[i][j]+(damping*max(H[i][i], 1e-6) if i == j else 0.0)
                  for j in range(5)] for i in range(5)]
            try:
                step = _solve(A, [-v for v in gradient])
                candidate = [p[i]+step[i] for i in range(5)]
                new_cost = _cost(_evaluate(candidate, samples, radius, focal)[0])
            except (ValueError, OverflowError, ZeroDivisionError):
                new_cost = 1e30
            if new_cost < old_cost:
                p, damping, improved = candidate, max(1e-8, damping/3), True
                break
            damping *= 8
        if not improved or sum(v*v for v in step) < 1e-12 or old_cost-new_cost < 1e-8:
            break
    return p, _cost(_evaluate(p, samples, radius, focal)[0])


def _reproject(camera, p, samples, radius):
    C = [radius*v for v in p[:3]]
    N = [p[3], p[4], -1.0]
    d = dot(N, C)
    errors, vectors = [], []
    for ray, known_radius, pixel in samples:
        s = d/dot(N, ray)
        q = [s*ray[i]-C[i] for i in range(3)]
        q = unit(q)
        predicted = [C[i]+known_radius*q[i] for i in range(3)]
        u, v = camera.project_ray(predicted)
        errors.append(math.sqrt((u-pixel[0])**2+(v-pixel[1])**2))
        vectors.append(q)
    return errors, vectors


def _coverage(vectors, normal):
    ex = unit(cross(normal, [0, 1, 0] if abs(normal[1]) < 0.9 else [1, 0, 0]))
    ey = cross(normal, ex)
    angles = sorted(math.atan2(dot(v, ey), dot(v, ex)) for v in vectors)
    if len(angles) < 16:
        return 0.0
    gaps = [angles[i+1]-angles[i] for i in range(len(angles)-1)]
    gaps.append(angles[0]+2*math.pi-angles[-1])
    return 360-max(gaps)*180/math.pi


def fit_target_center(camera, outer_points_px, inner_points_px,
                      outer_diameter_mm=100.0, inner_diameter_mm=55.0):
    """Fit common physical center + normal to both known circles.

    Several normal seeds are evaluated; competing low-error centers and poor
    local position observability cause rejection. In-plane roll is never made up.
    """
    try:
        if not all(finite(v) for v in (outer_diameter_mm, inner_diameter_mm)) or not 0 < inner_diameter_mm < outer_diameter_mm:
            raise ValueError("INVALID_TARGET_DIMENSIONS")
        outer, inner = list(outer_points_px), list(inner_points_px)
        if not 24 <= len(outer) <= 192 or not 20 <= len(inner) <= 192:
            raise ValueError("TWO_COMPLETE_RINGS_REQUIRED")
        radius, inner_radius = outer_diameter_mm/2.0, inner_diameter_mm/2.0
        outer_rays, inner_rays = [camera.ray(p) for p in outer], [camera.ray(p) for p in inner]
        samples = [(ray, radius, pixel) for ray, pixel in zip(outer_rays, outer)]
        samples += [(ray, inner_radius, pixel) for ray, pixel in zip(inner_rays, inner)]
        xs, ys = [v[0]/v[2] for v in outer_rays], [v[1]/v[2] for v in outer_rays]
        mx, my = sum(xs)/len(xs), sum(ys)/len(ys)
        xx = sum((x-mx)**2 for x in xs)/len(xs)
        yy = sum((y-my)**2 for y in ys)/len(ys)
        xy = sum((x-mx)*(y-my) for x, y in zip(xs, ys))/len(xs)
        largest = 0.5*(xx+yy+math.sqrt((xx-yy)**2+4*xy*xy))
        angular_radius = math.sqrt(2*largest)
        if boundary_checks() and angular_radius*min(camera.fx, camera.fy) < settings.MIN_OUTER_RADIUS_PX:
            raise ValueError("TARGET_TOO_SMALL")
        z = 1/angular_radius
        z = min(settings.MAX_DEPTH_MM/radius, max(settings.MIN_DEPTH_MM/radius, z))
        focal = math.sqrt(camera.fx*camera.fy)
        candidates = []
        for a, b in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1), (0.8, 0.8),
                     (0.8, -0.8), (-0.8, 0.8), (-0.8, -0.8)):
            try:
                p, cost = _refine([mx*z, my*z, z, a, b], samples, radius, focal)
                if not any(norm([radius*(p[i]-q[0][i]) for i in range(3)]) < 0.05
                           and norm([p[i]-q[0][i] for i in (3, 4)]+[0]) < 0.001 for q in candidates):
                    candidates.append((p, cost))
            except (ValueError, ZeroDivisionError, OverflowError):
                pass
        if not candidates:
            raise ValueError("TARGET_POSE_NOT_CONVERGED")
        candidates.sort(key=lambda item: item[1])
        p, cost = candidates[0]
        C = [radius*v for v in p[:3]]
        normal = unit([p[3], p[4], -1.0])
        competing = []
        for other, other_cost in candidates[1:]:
            if other_cost-cost > 9*settings.CORNER_NOISE_PX**2:
                continue
            delta = norm([radius*(other[i]-p[i]) for i in range(3)])
            competing.append({"center_difference_mm": delta})
            if delta > 3.0:
                raise ValueError("TARGET_CENTER_AMBIGUOUS")
        errors, vectors = _reproject(camera, p, samples, radius)
        limit = settings.MAX_REPROJECTION_PX
        indices = [i for i, e in enumerate(errors) if e <= limit]
        strict=boundary_checks()
        inlier_ratio=len(indices)/float(len(samples))
        if strict and len(indices) < int(math.ceil(settings.MIN_INLIER_RATIO*len(samples))):
            raise ValueError("TARGET_RINGS_INCONSISTENT")
        if not strict:indices=list(range(len(samples)))
        outer_inliers = [i for i in indices if i < len(outer)]
        inner_inliers = [i for i in indices if i >= len(outer)]
        coverage = [_coverage([vectors[i] for i in group], normal) for group in (outer_inliers, inner_inliers)]
        if strict and min(coverage) < settings.MIN_COVERAGE_DEG:
            raise ValueError("TARGET_RING_INCOMPLETE")
        rms = math.sqrt(sum(errors[i]**2 for i in indices)/len(indices))
        _, rows = _evaluate(p, [samples[i] for i in indices], radius, focal, True)
        H, _, _ = _normal_equations([0.0]*len(rows), rows, False)
        inverse = [_solve(H, [float(i == j) for i in range(5)]) for j in range(3)]
        sigma = max(settings.CORNER_NOISE_PX, rms)
        diagonal = [inverse[i][i] for i in range(3)]
        if min(diagonal) < 0:
            raise ValueError("TARGET_GEOMETRY_UNOBSERVABLE")
        position_3sigma = 3*sigma*radius*math.sqrt(sum(diagonal[:3]))
        if position_3sigma > settings.MAX_POSITION_3SIGMA_MM:
            raise ValueError("TARGET_POSITION_POORLY_OBSERVABLE")
        return {"ok": True, "reason": "OK", "position_camera_mm": C,
                "orientation_valid": False,
                "in_plane_rotation_observable": False, "reference_point": "target_physical_center",
                "quality": {"reprojection_rms_px": rms, "inlier_ratio": inlier_ratio,
                            "boundary_checks_enabled":strict,
                            "warnings":(["TARGET_RINGS_INCONSISTENT"] if inlier_ratio<settings.MIN_INLIER_RATIO else [])+(["TARGET_RING_INCOMPLETE"] if min(coverage)<settings.MIN_COVERAGE_DEG else []),
                            "coverage_deg": coverage, "position_3sigma_mm": position_3sigma,
                            "candidate_count": len(candidates),
                            "competing": competing, "uniqueness_guaranteed": False,
                            "outer_diameter_mm": outer_diameter_mm, "inner_diameter_mm": inner_diameter_mm}}
    except (ValueError, TypeError, IndexError, ZeroDivisionError, OverflowError) as exc:
        return {"ok": False, "reason": str(exc), "position_camera_mm": None,
                "orientation_valid": False}
