"""Known rectangular table -> real slot reference, reusing verified QR pose math.

Only object coordinates and their initial homography/evaluation are specialized.
Model tilt is only used to locate physical table points. No heading is exported.
"""
import math
import rescue_config as settings
from geometry import finite, transform_point
from qr_pose import QRPoseEstimator, _solve, _orthogonal, _dot, _cross


class RescueTablePoseEstimator(QRPoseEstimator):
    def __init__(self, camera, width_mm, depth_mm):
        if not finite(width_mm) or not finite(depth_mm) or not 0 < depth_mm < width_mm:
            raise ValueError("INVALID_TABLE_DIMENSIONS")
        self.width_mm, self.depth_mm = width_mm, depth_mm
        half_depth = depth_mm/(2.0*width_mm)
        # Far-left, far-right, near-right, near-left in the physical table frame.
        self.object_corners = ((-.5, half_depth), (.5, half_depth),
                               (.5, -half_depth), (-.5, -half_depth))
        super().__init__(width_mm, camera.K, camera.D, camera.image_size,
                         max_reprojection_error_px=settings.TABLE_REPROJECTION_PX,
                         corner_noise_px=settings.TABLE_CORNER_NOISE_PX,
                         max_position_uncertainty_mm=settings.MAX_REFERENCE_3SIGMA_MM)

    def _homography_pose(self, uv):
        mx, my = sum(p[0] for p in uv)/4.0, sum(p[1] for p in uv)/4.0
        scale = math.sqrt(sum((p[0]-mx)**2+(p[1]-my)**2 for p in uv)/4.0)
        if scale < 1e-8:
            raise ValueError("degenerate_table_corners")
        A, b = [], []
        for (x, y), (u, v) in zip(self.object_corners, uv):
            u, v = (u-mx)/scale, (v-my)/scale
            A.append([x,y,1.0,0.0,0.0,0.0,-u*x,-u*y]); b.append(u)
            A.append([0.0,0.0,0.0,x,y,1.0,-v*x,-v*y]); b.append(v)
        h = _solve(A,b)+[1.0]
        h = [scale*h[j]+mx*h[6+j] for j in range(3)]+[
             scale*h[3+j]+my*h[6+j] for j in range(3)]+h[6:9]
        c1,c2 = [h[0],h[3],h[6]], [h[1],h[4],h[7]]
        lam = 2.0/(math.sqrt(_dot(c1,c1))+math.sqrt(_dot(c2,c2)))
        R = _orthogonal([[c1[i]*lam,c2[i]*lam,0.0] for i in range(3)])
        return R,[h[2]*lam,h[5]*lam,lam]

    def _evaluate(self,R,t,corners,jacobian=False):
        residual,rows = [],[]
        for (xo,yo),(u,v) in zip(self.object_corners,corners):
            q = [R[i][0]*xo+R[i][1]*yo for i in range(3)]
            p = [q[i]+t[i] for i in range(3)]
            if not all(finite(x) for x in p) or p[2] <= 1e-6:
                raise ValueError("nonpositive_table_depth")
            x,y = p[0]/p[2],p[1]/p[2]
            if max(abs(x),abs(y)) > 3:
                raise ValueError("table_projection_out_of_range")
            xd,yd,a,b,c,d = self._distort(x,y)
            residual.extend((u-(self.fx*xd+self.cx),v-(self.fy*yd+self.cy)))
            if jacobian:
                for focal,dx,dy in ((self.fx,a,b),(self.fy,c,d)):
                    g = [focal*dx/p[2],focal*dy/p[2],-focal*(dx*x+dy*y)/p[2]]
                    rows.append(_cross(q,g)+g)
        if not all(finite(x) for x in residual):
            raise ValueError("nonfinite_table_projection")
        return sum(x*x for x in residual),residual,rows

    def estimate_table(self,corners_px,geometry_confirmed=False,physical_order_confirmed=False):
        if not geometry_confirmed:
            return {"ok":False,"reason":"RAW_GEOMETRY_NOT_CONFIRMED","T_camera_from_table":None}
        if not physical_order_confirmed:
            return {"ok":False,"reason":"TABLE_CORNER_ORDER_NOT_CONFIRMED","T_camera_from_table":None}
        pose = self.estimate_from_corners(corners_px,actual_image_size=self.image_size)
        if not pose["ok"]:
            return {"ok":False,"reason":"TABLE_"+pose["reason"].upper(),"T_camera_from_table":None}
        try:
            R,t = pose["R_camera_from_qr"],[v/self.size for v in pose["t_camera_from_qr_mm"]]
            _,residual,rows = self._evaluate(R,t,corners_px,True)
            A,_,scales = self._normal(rows,residual)
            inverse = [_solve(A,[float(i==j) for i in range(6)]) for j in range(6)]
            covariance = [[inverse[j][i]*self.noise*self.noise/(scales[i]*scales[j])
                           for j in range(6)] for i in range(6)]
            return {"ok":True,"reason":"OK","T_camera_from_table":pose["T_camera_from_qr"],
                    "corners_px":pose["corners_px"],"table_width_mm":self.width_mm,
                    "table_depth_mm":self.depth_mm,"_pose_covariance":covariance,
                    "_candidate_models":pose["_candidate_models"],
                    "quality":{"reprojection_rms_px":pose["reprojection_error_px"],
                               "position_3sigma_mm":pose["position_3sigma_mm"],
                               "candidate_count":pose["candidate_count"],"uniqueness_guaranteed":False}}
        except (ValueError,ZeroDivisionError,OverflowError) as exc:
            return {"ok":False,"reason":"TABLE_"+str(exc).upper(),"T_camera_from_table":None}


def project_table_point(camera,table_pose,point_table_mm):
    """Point location and local assumed-noise uncertainty; no bbox triangulation."""
    if not table_pose.get("ok"):
        raise ValueError("TABLE_POSE_INVALID")
    T = table_pose["T_camera_from_table"]
    point = transform_point(T,point_table_mm)
    for model_r,model_t in table_pose.get("_candidate_models",[]):
        alternate=[sum(model_r[i][j]*point_table_mm[j] for j in range(3))+model_t[i]*table_pose["table_width_mm"] for i in range(3)]
        if math.sqrt(sum((alternate[i]-point[i])**2 for i in range(3))) > settings.MAX_REFERENCE_3SIGMA_MM:
            raise ValueError("TABLE_POINT_AMBIGUOUS")
    if point[2] <= 0:
        raise ValueError("TABLE_POINT_BEHIND_CAMERA")
    scale = table_pose["table_width_mm"]
    q = [sum(T[i][j]*point_table_mm[j] for j in range(3)) for i in range(3)]
    x,y,z = q
    J = [[0,z,-y,scale,0,0],[-z,0,x,0,scale,0],[y,-x,0,0,0,scale]]
    covariance = table_pose["_pose_covariance"]
    variance = sum(sum(J[k][i]*covariance[i][j]*J[k][j] for i in range(6) for j in range(6)) for k in range(3))
    if not finite(variance) or variance < -1e-6:
        raise ValueError("TABLE_POINT_UNOBSERVABLE")
    uncertainty = 3*math.sqrt(max(0.0,variance))
    if uncertainty > settings.MAX_REFERENCE_3SIGMA_MM:
        raise ValueError("TABLE_POINT_POORLY_OBSERVABLE")
    return {"position_camera_mm":point,"pixel_raw":camera.project_ray(point),
            "position_3sigma_mm":uncertainty}
