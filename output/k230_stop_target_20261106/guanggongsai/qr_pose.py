"""普通 QR 的已知尺寸中心定位；模型倾斜仅用于透视测距，不验收/回传航向。

C: X 向图像右、Y 向下、Z 向镜头前；Q: X 自身向右、Y 向上、Z 向正面外。
P_C = R_camera_from_qr * P_Q + t_camera_from_qr_mm，全部长度为 mm。
输入必须是原始畸变图上的四角，严格按 QR 自身 TL,TR,BR,BL；不按图像排序。
本文件导入时无硬件副作用。相机与显示见 qr_pose_example.py。
"""
import math
from boundary_policy import enabled as boundary_checks

IMAGE_SIZE = (640, 480)
CAMERA_MATRIX = [
    [540.241698997, 0.0, 320.099137389],
    [0.0, 719.860920175, 248.378807146],
    [0.0, 0.0, 1.0]
]
DIST_COEFFS = [0.071451906, -0.206368008, -0.001534711, 0.001398159, 0.0]
_SQUARE = ((-0.5, 0.5), (0.5, 0.5), (0.5, -0.5), (-0.5, -0.5))
_RAD = math.pi / 180.0


def _finite(x):
    # 不依赖某些固件缺少的 math.isfinite。
    return x == x and abs(x) < 1e30


def _dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def _cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def _unit(a):
    n = math.sqrt(_dot(a, a))
    if not _finite(n) or n < 1e-10:
        raise ValueError("degenerate_vector")
    return [x / n for x in a]


def _mm(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _orthogonal(r):
    # Gram-Schmidt 加右手叉积，使 R^T R=I、det(R)=+1。
    a = _unit([r[i][0] for i in range(3)])
    b = [r[i][1] for i in range(3)]
    ab = _dot(a, b)
    b = _unit([b[i]-ab*a[i] for i in range(3)])
    c = _cross(a, b)
    return [[a[i], b[i], c[i]] for i in range(3)]


def _exp(w):
    # SO(3) 指数映射，w 是弧度旋转增量，绝不是欧拉角。
    a2 = _dot(w, w)
    if a2 < 1e-8:
        s, c = 1.0-a2/6.0, 0.5-a2/24.0
    else:
        a = math.sqrt(a2)
        s, c = math.sin(a)/a, (1.0-math.cos(a))/a2
    x, y, z = w
    k = [[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]]
    return [[(1.0 if i == j else 0.0) + s*k[i][j] +
             c*(w[i]*w[j]-(a2 if i == j else 0.0)) for j in range(3)] for i in range(3)]


def _solve(a, b):
    # 小型线性方程：逐行缩放、部分主元消去；不依赖 numpy/ulab。
    n = len(b)
    m = []
    for i in range(n):
        scale = max(abs(x) for x in a[i])
        if not _finite(scale) or scale < 1e-20:
            raise ValueError("singular_system")
        m.append([x/scale for x in a[i]] + [b[i]/scale])
    for k in range(n):
        p = max(range(k, n), key=lambda i: abs(m[i][k]))
        if abs(m[p][k]) < 1e-7:
            raise ValueError("ill_conditioned_system")
        m[k], m[p] = m[p], m[k]
        for i in range(k+1, n):
            f = m[i][k]/m[k][k]
            for j in range(k+1, n+1):
                m[i][j] -= f*m[k][j]
            m[i][k] = 0.0
    x = [0.0]*n
    for i in range(n-1, -1, -1):
        x[i] = (m[i][n]-sum(m[i][j]*x[j] for j in range(i+1, n)))/m[i][i]
    if not all(_finite(v) for v in x):
        raise ValueError("nonfinite_solution")
    return x


class QRPoseEstimator:
    def __init__(self, qr_size_mm, camera_matrix=CAMERA_MATRIX,
                 dist_coeffs=DIST_COEFFS, image_size=IMAGE_SIZE,
                 detector_order=None, corner_adapter=None,
                 max_reprojection_error_px=2.0, corner_noise_px=0.5,
                 max_center_disagreement_mm=8.0, max_position_uncertainty_mm=25.0,
                 min_edge_px=8.0, max_iterations=35):
        """qr_size_mm 必填且 >0；测量编码正方形边界，不含外围白色静区。

        detector_order 默认 None：detect() 不信任未经核实的检测器方向。
        核实为 CanMV quirc 网格角点顺序后，设为 'canmv_quirc'。
        其他固件可提供 corner_adapter(img, code)，返回自身 TL,TR,BR,BL
        原图角点；不能可靠确定方向时必须返回 None。
        corner_noise_px 为每个像素坐标的预期标准差，用于保守歧义门限。
        """
        try:
            if isinstance(qr_size_mm, bool):
                raise ValueError()
            self.size = float(qr_size_mm)
            if not _finite(self.size) or self.size <= 0.0:
                raise ValueError()
        except (TypeError, ValueError, OverflowError):
            raise ValueError("qr_size_mm must be a finite positive measured size in mm")
        try:
            k = [[float(x) for x in row] for row in camera_matrix]
            d = [float(x) for x in dist_coeffs]
            if len(k) != 3 or any(len(row) != 3 for row in k) or len(d) != 5:
                raise ValueError()
            if not all(_finite(x) for row in k for x in row) or not all(_finite(x) for x in d):
                raise ValueError()
            if k[0][0] <= 0 or k[1][1] <= 0 or any(abs(x) > 1e-8 for x in
                    (k[0][1], k[1][0], k[2][0], k[2][1], k[2][2]-1.0)):
                raise ValueError()
            self.image_size = tuple(int(x) for x in image_size)
            if len(self.image_size) != 2 or min(self.image_size) <= 0 or tuple(image_size) != self.image_size:
                raise ValueError()
        except (TypeError, ValueError, IndexError, OverflowError):
            raise ValueError("invalid K (zero skew required), five-term D, or image_size")
        self.fx, self.fy, self.cx, self.cy = k[0][0], k[1][1], k[0][2], k[1][2]
        self.d = d
        if detector_order not in (None, "canmv_quirc"):
            raise ValueError("unknown detector_order; use corner_adapter for other detectors")
        if corner_adapter is not None and not callable(corner_adapter):
            raise ValueError("corner_adapter must be callable")
        self.detector_order, self.corner_adapter = detector_order, corner_adapter
        values = tuple(float(x) for x in (max_reprojection_error_px, corner_noise_px,
                       max_center_disagreement_mm, max_position_uncertainty_mm, min_edge_px))
        if not all(_finite(x) and x > 0 for x in values):
            raise ValueError("thresholds and corner_noise_px must be finite and positive")
        self.max_error, self.noise, self.max_center_difference, self.max_position_uncertainty, self.min_edge = values
        self.iterations = int(max_iterations)
        if self.iterations < 5 or self.iterations > 100:
            raise ValueError("max_iterations must be in 5..100")

    def _distort(self, x, y):
        # Brown-Conrady / OpenCV 五参数模型，同时返回解析导数。
        k1, k2, p1, p2, k3 = self.d
        r2 = x*x+y*y
        radial = 1.0+r2*(k1+r2*(k2+r2*k3))
        g = k1+r2*(2.0*k2+3.0*k3*r2)
        rx, ry = 2.0*x*g, 2.0*y*g
        xd = x*radial+2.0*p1*x*y+p2*(r2+2.0*x*x)
        yd = y*radial+p1*(r2+2.0*y*y)+2.0*p2*x*y
        return (xd, yd, radial+x*rx+2*p1*y+6*p2*x,
                x*ry+2*p1*x+2*p2*y, y*rx+2*p1*x+2*p2*y,
                radial+y*ry+6*p1*y+2*p2*x)

    def _undistort(self, u, v):
        # 输出无畸变归一化坐标 (X/Z,Y/Z)，不是像素，后续单应不再用 K。
        xd, yd = (u-self.cx)/self.fx, (v-self.cy)/self.fy
        x, y = xd, yd
        for _ in range(15):
            a, b, j00, j01, j10, j11 = self._distort(x, y)
            ex, ey = a-xd, b-yd
            if max(abs(ex)*self.fx, abs(ey)*self.fy) < 0.0002:
                return (x, y)
            det = j00*j11-j01*j10
            if not _finite(det) or det < 1e-8:
                raise ValueError("undistortion_singular")
            dx, dy = (j11*ex-j01*ey)/det, (-j10*ex+j00*ey)/det
            # 回溯，避免 Newton 在不适用的标定范围内跑向远处的根。
            accepted = False
            for _ in range(8):
                nx, ny = x-dx, y-dy
                if _finite(nx) and _finite(ny) and max(abs(nx), abs(ny)) < 3:
                    a, b = self._distort(nx, ny)[:2]
                    if (a-xd)**2+(b-yd)**2 < ex*ex+ey*ey:
                        x, y, accepted = nx, ny, True
                        break
                dx, dy = dx*0.5, dy*0.5
            if not accepted:
                break
        a, b = self._distort(x, y)[:2]
        if max(abs(a-xd)*self.fx, abs(b-yd)*self.fy) > 0.002:
            raise ValueError("undistortion_did_not_converge")
        return (x, y)

    def _homography_pose(self, uv):
        # 物体坐标暂以边长为 1；同时居中/缩放像点，改善 float32 条件数。
        mx = sum(p[0] for p in uv)/4.0
        my = sum(p[1] for p in uv)/4.0
        scale = math.sqrt(sum((p[0]-mx)**2+(p[1]-my)**2 for p in uv)/4.0)
        if scale < 1e-8:
            raise ValueError("degenerate_normalized_corners")
        a, b = [], []
        for (x, y), (u, v) in zip(_SQUARE, uv):
            u, v = (u-mx)/scale, (v-my)/scale
            a.append([x, y, 1.0, 0.0, 0.0, 0.0, -u*x, -u*y]); b.append(u)
            a.append([0.0, 0.0, 0.0, x, y, 1.0, -v*x, -v*y]); b.append(v)
        h = _solve(a, b) + [1.0]
        # 撤销像点归一化，H: (X/L,Y/L,1) -> (x_normal,y_normal,1)。
        h = [scale*h[j]+mx*h[6+j] for j in range(3)] + [
             scale*h[3+j]+my*h[6+j] for j in range(3)] + h[6:9]
        c1, c2 = [h[0], h[3], h[6]], [h[1], h[4], h[7]]
        lam = 2.0/(math.sqrt(_dot(c1, c1))+math.sqrt(_dot(c2, c2)))
        # h33=1，故 t_z 为正；另一整体符号会把全部角点放到相机后方。
        r = _orthogonal([[c1[i]*lam, c2[i]*lam, 0.0] for i in range(3)])
        return r, [h[2]*lam, h[5]*lam, lam]

    def _evaluate(self, r, t, corners, jacobian=False):
        residual, rows = [], []
        for (xq, yq), (u, v) in zip(_SQUARE, corners):
            q = [r[i][0]*xq+r[i][1]*yq for i in range(3)]
            p = [q[i]+t[i] for i in range(3)]
            if not all(_finite(x) for x in p) or p[2] <= 1e-6:
                raise ValueError("nonpositive_corner_depth")
            x, y = p[0]/p[2], p[1]/p[2]
            if max(abs(x), abs(y)) > 3.0:
                raise ValueError("projection_out_of_model_range")
            xd, yd, a, b, c, d = self._distort(x, y)
            residual.extend((u-(self.fx*xd+self.cx), v-(self.fy*yd+self.cy)))
            if jacobian:
                for fx, dx, dy in ((self.fx, a, b), (self.fy, c, d)):
                    g = [fx*dx/p[2], fx*dy/p[2], -fx*(dx*x+dy*y)/p[2]]
                    # R <- exp(dw) R，t 单独更新；旋转导数为 q × g。
                    rows.append(_cross(q, g)+g)
        if not all(_finite(x) for x in residual):
            raise ValueError("nonfinite_projection")
        return sum(x*x for x in residual), residual, rows

    def _normal(self, rows, residual):
        scales = [math.sqrt(sum(row[j]*row[j] for row in rows)) for j in range(6)]
        if min(scales) < 1e-8:
            raise ValueError("unobservable_pose")
        j = [[row[k]/scales[k] for k in range(6)] for row in rows]
        a = [[sum(row[i]*row[k] for row in j) for k in range(6)] for i in range(6)]
        b = [sum(row[i]*e for row, e in zip(j, residual)) for i in range(6)]
        return a, b, scales

    def _refine(self, r, t, corners):
        damping = 0.001
        cost, residual, rows = self._evaluate(r, t, corners, True)
        converged = False
        for _ in range(self.iterations):
            a, b, scales = self._normal(rows, residual)
            if max(abs(v) for v in b) < 0.00002 or cost < 1e-8:
                converged = True
                break
            accepted = False
            for _ in range(9):
                damped = [[a[i][j]+(damping if i == j else 0.0) for j in range(6)] for i in range(6)]
                step = _solve(damped, b)
                step = [step[i]/scales[i] for i in range(6)]
                # 防止单步跨过正深度边界或大的旋转分支。
                f = max(1.0, math.sqrt(_dot(step[:3], step[:3]))/0.35,
                        math.sqrt(_dot(step[3:], step[3:]))/max(0.1, 0.4*t[2]))
                step = [v/f for v in step]
                nr = _orthogonal(_mm(_exp(step[:3]), r))
                nt = [t[i]+step[i+3] for i in range(3)]
                try:
                    nc, ne, nj = self._evaluate(nr, nt, corners, True)
                except ValueError:
                    nc = 1e30
                if nc < cost:
                    improvement = cost-nc
                    r, t, cost, residual, rows = nr, nt, nc, ne, nj
                    damping = max(1e-7, damping*0.3)
                    accepted = True
                    if improvement < max(1e-8, cost*2e-6):
                        converged = True
                    break
                damping *= 10.0
            if converged:
                break
            if not accepted:
                # float32 在极值附近可能无法再减小残差；最后仍做梯度检查。
                converged = max(abs(v) for v in b) < 0.002
                break
        if not converged:
            _, b, _ = self._normal(rows, residual)
            converged = max(abs(v) for v in b) < 0.002
        if not converged:
            raise ValueError("optimizer_not_converged")
        return cost, r, t

    def _seeds(self, r, t):
        yield r, t
        # 关于中心视线反射法向，构造平面双解的另一侧初值。
        n = [r[i][2] for i in range(3)]
        v = _unit(t)
        n2 = [2.0*_dot(n, v)*v[i]-n[i] for i in range(3)]
        axis = _cross(n, n2)
        sn, cs = math.sqrt(_dot(axis, axis)), _dot(n, n2)
        if sn > 1e-6:
            angle = math.atan2(sn, cs)
            mirror = _mm(_exp([a*angle/sn for a in axis]), r)
            yield mirror, t
        # 从多个倾角寻找不同局部极小；有限多起点不构成全局唯一性证明。
        for angle in (-0.55, 0.55):
            yield _mm(_exp([angle, 0.0, 0.0]), r), t
            yield _mm(_exp([0.0, angle, 0.0]), r), t

    def _position_uncertainty(self, r, t, corners):
        """Translation covariance with model tilt as nuisance parameters, mm.

        No rotation confidence or angular acceptance is computed. The three
        translation columns of the inverse normal matrix retain coupling to
        tilt; treating a fitted plane as perfectly known would understate error.
        """
        _, residual, rows = self._evaluate(r, t, corners, True)
        a, _, scales = self._normal(rows, residual)
        variance = 0.0
        for i in range(3, 6):
            b = [0.0]*6; b[i] = 1.0
            diagonal = _solve(a, b)[i]/(scales[i]*scales[i])
            if diagonal <= 0 or not _finite(diagonal):
                raise ValueError("position_unobservable")
            variance += diagonal*self.noise*self.noise*self.size*self.size
        return 3.0*math.sqrt(variance)

    def _failure(self, reason, corners=None, payload=None, **extra):
        result = {"ok": False, "reason": reason, "payload": payload, "corners_px": corners}
        result.update(extra)
        return result

    def estimate_from_corners(self, corners, actual_image_size=None):
        """输入原始畸变图的物理 TL,TR,BR,BL；不得输入 rect() 外接框。

        调用者负责确认视场/裁剪/翻转/缩放与标定完全一致。
        actual_image_size 可用于额外检查，未知的几何变化不能仅靠图像尺寸检测。
        返回字典；失败不返回伪造的 R/t/T。正面 QR、非镜像图是本模块前提。
        """
        points = None
        try:
            if actual_image_size is not None and tuple(actual_image_size) != self.image_size:
                raise ValueError("image_size_mismatch")
            if len(corners) != 4 or any(len(p) != 2 for p in corners):
                raise ValueError("need_four_2d_corners")
            points = [(float(p[0]), float(p[1])) for p in corners]
            width, height = self.image_size
            if not all(_finite(x) for p in points for x in p):
                raise ValueError("nonfinite_corners")
            if not all(0 <= u < width and 0 <= v < height for u, v in points):
                raise ValueError("corners_outside_calibrated_image")
            crosses = []
            for i in range(4):
                a, b, c = points[i], points[(i+1)%4], points[(i+2)%4]
                ex, ey, fx, fy = b[0]-a[0], b[1]-a[1], c[0]-b[0], c[1]-b[1]
                if ex*ex+ey*ey < (self.min_edge*self.min_edge if boundary_checks() else 1e-8):
                    raise ValueError("qr_too_small_or_duplicate_corners")
                crosses.append(ex*fy-ey*fx)
            if min(crosses) <= 1e-4:
                raise ValueError("nonconvex_mirrored_or_wrong_corner_winding")
            area = 0.5*sum(points[i][0]*points[(i+1)%4][1]-points[i][1]*points[(i+1)%4][0] for i in range(4))
            span2 = max((a[0]-b[0])**2+(a[1]-b[1])**2 for a in points for b in points)
            if area < 32.0 or area/span2 < 0.015 or min(crosses)/span2 < 0.002:
                raise ValueError("degenerate_or_edge_on_quadrilateral")
            r, t = self._homography_pose([self._undistort(u, v) for u, v in points])
            candidates, failed_starts = [], 0
            for sr, st in self._seeds(r, t):
                try:
                    candidate = self._refine(sr, st, points)
                    cr, ct = candidate[1:]
                    # Deduplicate identical numerical models only. No heading
                    # difference is computed or used to accept a position.
                    if not any(sum((cr[i][j]-x[1][i][j])**2 for i in range(3) for j in range(3)) < 1e-10 and
                               sum((ct[i]-x[2][i])**2 for i in range(3)) < 1e-10 for x in candidates):
                        candidates.append(candidate)
                except (ValueError, OverflowError, ZeroDivisionError):
                    failed_starts += 1
            if not candidates:
                raise ValueError("no_converged_position_model")
            candidates.sort(key=lambda x: x[0])
            cost, r, t = candidates[0]
            error = math.sqrt(cost/4.0)  # 四角二维欧氏残差平方的均方根，单位像素。
            if boundary_checks() and error > self.max_error:
                return self._failure("reprojection_error_too_large", points,
                                     reprojection_error_px=error)
            # Only center disagreement is relevant to this application. Two
            # plane models may face differently while locating the same center.
            plausible=[item for item in candidates if item[0]-cost <= 9.0*self.noise*self.noise]
            center_difference=max(math.sqrt(sum(((item[2][i]-t[i])*self.size)**2 for i in range(3)))
                                  for item in plausible)
            if center_difference > self.max_center_difference:
                return self._failure("qr_center_ambiguous", points,
                    center_disagreement_mm=center_difference)
            uncertainty=max(self._position_uncertainty(rr,tt,points) for _,rr,tt in plausible)
            if uncertainty > self.max_position_uncertainty:
                return self._failure("qr_position_uncertain", points,
                    position_3sigma_mm=uncertainty)
            tm = [v*self.size for v in t]
            if not all(_finite(v) for v in tm) or not _finite(_dot(tm, tm)):
                raise ValueError("nonfinite_metric_translation")
            transform = [r[i][:]+[tm[i]] for i in range(3)]+[[0.0, 0.0, 0.0, 1.0]]
            return {"ok": True, "reason": "ok", "payload": None, "corners_px": points,
                    "R_camera_from_qr": r, "t_camera_from_qr_mm": tm,
                    "T_camera_from_qr": transform, "depth_mm": tm[2],
                    "distance_mm": math.sqrt(_dot(tm, tm)),
                    "reprojection_error_px": error,
                    "boundary_checks_enabled":boundary_checks(),
                    "boundary_warning":error>self.max_error,
                    "position_camera_mm": tm, "position_3sigma_mm": uncertainty,
                    "center_disagreement_mm": center_difference,
                    "_candidate_models": [(rr,tt) for _,rr,tt in plausible],
                    "candidate_count": len(candidates),
                    "failed_starts": failed_starts}
        except (ValueError, TypeError, IndexError, OverflowError, ZeroDivisionError) as exc:
            return self._failure(str(exc), points)

    def detect(self, img, target_payload=None, roi=None, geometry_confirmed=False):
        """全图或原图 ROI 检测；返回列表，未检测到目标则 []。

        geometry_confirmed=True 表示调用者确认这是标定对应的原始图像：
        同一传感器模式、裁剪、缩放、翻转、视场与焦距，未事先去畸变。
        仅检查 width/height 无法识别同尺寸不同视场，所以需显式声明。
        CanMV quirc 的 ROI 输出已经是原图坐标，本方法不重复加 ROI 偏移。
        不支持先裁出一张新图再传入；这种情况需外部还原坐标后调用核心。
        """
        if (img.width(), img.height()) != self.image_size:
            raise ValueError("image_size_mismatch; use the calibrated raw image")
        if roi is not None:
            if len(roi) != 4 or any(int(x) != x for x in roi):
                raise ValueError("roi must be integer (x,y,w,h)")
            x, y, w, h = roi
            if x < 0 or y < 0 or w <= 0 or h <= 0 or x+w > self.image_size[0] or y+h > self.image_size[1]:
                raise ValueError("roi outside image")
            codes = img.find_qrcodes(roi=roi)
        else:
            codes = img.find_qrcodes()
        results = []
        for code in codes:
            payload = code.payload()
            if target_payload is not None and payload != target_payload:
                continue  # 精确匹配，不擅自 strip 或修改二维码内容。
            corners = None
            try:
                corners = code.corners()
                if not geometry_confirmed:
                    result = self._failure("calibrated_raw_geometry_not_confirmed", corners)
                elif self.corner_adapter is not None:
                    corners = self.corner_adapter(img, code)
                    result = (self._failure("corner_orientation_unknown") if corners is None
                              else self.estimate_from_corners(corners))
                elif self.detector_order == "canmv_quirc":
                    result = self.estimate_from_corners(corners)
                else:
                    result = self._failure("corner_orientation_unverified", corners)
            except (ValueError, TypeError, AttributeError, IndexError) as exc:
                result = self._failure("qr_interface_or_adapter_error: "+str(exc), corners)
            result["payload"] = payload
            results.append(result)
        return results
