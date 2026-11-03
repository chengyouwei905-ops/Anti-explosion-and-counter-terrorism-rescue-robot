"""Existing QR solver -> commanded gripper-center frame. No communication."""
from qr_pose import QRPoseEstimator
from config import COLOR_NAMES, QR_SIZE_MM, REPORT_CANDIDATE_TRANSFORMS


def parse_task(payload):
    # Same strict semantics as the verified recognition/task01_qr.py parser.
    if not isinstance(payload, str) or len(payload) != 3 or any(c not in "123" for c in payload):
        raise ValueError("INVALID_TASK_CODE")
    a, b, c = [int(v) for v in payload]
    return {"code": payload, "bomb_color_id": a, "target_color_id": b, "rescue_shape_id": c,
            "bomb_color": COLOR_NAMES[a], "target_color": COLOR_NAMES[b],
            "rescue_shape": (None, "cylinder", "cone", "drum")[c]}


class QRTask:
    def __init__(self, calibration, high_resolution_size=None):
        self.calibration = calibration
        c = calibration.camera
        self.estimator = QRPoseEstimator(QR_SIZE_MM, c.K, c.D, c.image_size, detector_order="canmv_quirc")
        self.high_resolution_estimator = None
        if high_resolution_size is not None:
            width,height=high_resolution_size
            if type(width) is not int or type(height) is not int or min(width,height)<=0:
                raise ValueError("INVALID_QR_CAPTURE_SIZE")
            sx,sy=width/float(c.image_size[0]),height/float(c.image_size[1])
            # 同一完整视场、无额外裁剪/翻转：K_new=diag(sx,sy,1)*K。
            # 640x480 -> 1920x1080为sx=3、sy=2.25，不能全部简单乘3。
            # 仅推导二维码内参，不修改calibration.camera、D或手眼矩阵。
            scaled_K=[[value*sx for value in c.K[0]],
                      [value*sy for value in c.K[1]],list(c.K[2])]
            self.high_resolution_estimator=QRPoseEstimator(
                QR_SIZE_MM,scaled_K,c.D,(width,height),detector_order="canmv_quirc")
        self.last_frame = -1

    def measure(self, raw_gray, frame_id, geometry_confirmed=False, stable=False):
        base = {"ok": False, "frame_id": frame_id, "decoded": False,
                "task_valid": False, "pose_valid": False, "control_ready": False,
                "T_tool_from_qr": None, "position_tool_mm": None}
        if type(frame_id) is not int or frame_id <= self.last_frame:
            base["reason"] = "STALE_OR_DUPLICATE_FRAME"
            return base
        self.last_frame = frame_id
        if not stable:
            base["reason"] = "OBSERVATION_NOT_STABLE"
            return base
        try:
            estimator=self.estimator
            if self.high_resolution_estimator is not None:
                size=(raw_gray.width(),raw_gray.height())
                if size==self.high_resolution_estimator.image_size:
                    estimator=self.high_resolution_estimator
            results = estimator.detect(raw_gray, geometry_confirmed=geometry_confirmed)
        except Exception as exc:
            base["reason"] = "QR_DETECTION_ERROR:"+str(exc)
            return base
        observations = []
        for result in results:
            obs = dict(base)
            obs.update(decoded=True, payload=result["payload"], pose_valid=result["ok"],
                       detection_image_size=estimator.image_size)
            try:
                obs["task"] = parse_task(result["payload"])
                obs["task_valid"] = True
            except ValueError:
                obs["task"] = None
            obs["pose_reason"] = result.get("reason", "OK")
            obs["reason"] = "INVALID_TASK_CODE" if not obs["task_valid"] else obs["pose_reason"]
            if result["ok"]:
                transformed = self.calibration.pose_to_tool(result["T_camera_from_qr"], REPORT_CANDIDATE_TRANSFORMS)
                obs.update(position_tool_mm=transformed["position_tool_mm"],
                           T_tool_from_qr=transformed["T_tool_from_target"],
                           T_camera_from_qr=result["T_camera_from_qr"],
                           reprojection_error_px=result["reprojection_error_px"],
                           coordinate_frame="E_commanded_gripper_center", units="mm",
                           handeye_accepted=self.calibration.handeye_accepted)
                obs["ok"] = obs["task_valid"] and transformed["ok"]
                obs["control_ready"] = obs["ok"] and transformed["control_ready"]
                if obs["task_valid"]:
                    obs["reason"] = transformed["reason"]
            observations.append(obs)
        if len(observations) == 1:
            return observations[0]
        base.update(reason="QR_NOT_FOUND" if not observations else "MULTIPLE_QRS_AMBIGUOUS",
                    decoded=bool(observations), observations=observations)
        return base
