"""Existing QR solver -> commanded gripper-center frame. No communication."""
from qr_pose import QRPoseEstimator
from config import (COLOR_NAMES, QR_SIZE_MM, REPORT_CANDIDATE_TRANSFORMS,
                    QR_MAX_CENTER_DISAGREEMENT_MM, QR_MAX_POSITION_3SIGMA_MM)


def parse_task(payload):
    # Same strict semantics as the verified recognition/task01_qr.py parser.
    if not isinstance(payload, str) or len(payload) != 3 or any(c not in "123" for c in payload):
        raise ValueError("INVALID_TASK_CODE")
    a, b, c = [int(v) for v in payload]
    return {"code": payload, "bomb_color_id": a, "target_color_id": b, "rescue_shape_id": c,
            "bomb_color": COLOR_NAMES[a], "target_color": COLOR_NAMES[b],
            "rescue_shape": (None, "cylinder", "cone", "drum")[c]}


class QRTask:
    def __init__(self, calibration, high_resolution_size=None, contrast_retry=False,
                 contrast_retry_every=3,roi_size=None,roi_reference_size=None):
        if type(contrast_retry_every) is not int or contrast_retry_every<1:
            raise ValueError("INVALID_QR_CONTRAST_RETRY_INTERVAL")
        self.calibration = calibration
        self.contrast_retry=bool(contrast_retry)
        self.contrast_retry_every=contrast_retry_every
        self.missed_detections=0
        self.roi_size=None if roi_size is None else tuple(roi_size)
        self.roi_reference_size=None if roi_reference_size is None else tuple(roi_reference_size)
        c = calibration.camera
        self.estimator = QRPoseEstimator(QR_SIZE_MM, c.K, c.D, c.image_size, detector_order="canmv_quirc",
            max_center_disagreement_mm=QR_MAX_CENTER_DISAGREEMENT_MM,
            max_position_uncertainty_mm=QR_MAX_POSITION_3SIGMA_MM)
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
                QR_SIZE_MM,scaled_K,c.D,(width,height),detector_order="canmv_quirc",
                max_center_disagreement_mm=QR_MAX_CENTER_DISAGREEMENT_MM,
                max_position_uncertainty_mm=QR_MAX_POSITION_3SIGMA_MM)
        self.last_frame = -1
        self.detection_roi(self.estimator.image_size)
        if self.high_resolution_estimator is not None:
            self.detection_roi(self.high_resolution_estimator.image_size)

    def detection_roi(self,image_size):
        if self.roi_size is None:return None
        if len(self.roi_size)!=2 or any(type(v) is not int or v<=0 for v in self.roi_size):
            raise ValueError("INVALID_QR_ROI_SIZE")
        iw,ih=image_size
        width,height=self.roi_size
        if self.roi_reference_size is not None:
            if (len(self.roi_reference_size)!=2 or
                    any(type(v) is not int or v<=0 for v in self.roi_reference_size)):
                raise ValueError("INVALID_QR_ROI_REFERENCE_SIZE")
            rw,rh=self.roi_reference_size
            width=max(1,int(round(width*iw/float(rw))))
            height=max(1,int(round(height*ih/float(rh))))
        if width>iw or height>ih:raise ValueError("QR_ROI_OUTSIDE_IMAGE")
        return ((iw-width)//2,(ih-height)//2,width,height)

    def measure(self, raw_gray, frame_id, geometry_confirmed=False, stable=False):
        base = {"ok": False, "frame_id": frame_id, "decoded": False,
                "task_valid": False, "pose_valid": False, "control_ready": False,
                "position_tool_mm": None}
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
            roi=self.detection_roi((raw_gray.width(),raw_gray.height()))
            base["detection_roi"]=roi
            results = estimator.detect(raw_gray,roi=roi,geometry_confirmed=geometry_confirmed)
        except Exception as exc:
            base["reason"] = "QR_DETECTION_ERROR:"+str(exc)
            return base
        detection_mode="raw"
        if results:
            self.missed_detections=0
        else:
            self.missed_detections+=1
            if (self.contrast_retry and geometry_confirmed and
                    self.missed_detections>=self.contrast_retry_every):
                self.missed_detections=0
                enhanced=None
                try:
                    # Change intensities only: full-size copy, no crop/resize or
                    # lens correction, so original calibrated pixel geometry holds.
                    enhanced=raw_gray.copy()
                    enhanced.histeq(adaptive=False)
                    results=estimator.detect(enhanced,roi=roi,geometry_confirmed=geometry_confirmed)
                    detection_mode="contrast"
                except Exception as exc:
                    # Optional enhancement failure must not break raw detection.
                    base["contrast_retry_error"]=str(exc)
                finally:
                    enhanced=None
        observations = []
        for result in results:
            obs = dict(base)
            obs.update(decoded=True, payload=result["payload"], pose_valid=result["ok"],
                       detection_image_size=estimator.image_size,detection_mode=detection_mode)
            try:
                obs["task"] = parse_task(result["payload"])
                obs["task_valid"] = True
            except ValueError:
                obs["task"] = None
            obs["pose_reason"] = result.get("reason", "OK")
            obs["reason"] = "INVALID_TASK_CODE" if not obs["task_valid"] else obs["pose_reason"]
            if result["ok"]:
                transformed = self.calibration.point_to_tool(result["position_camera_mm"], REPORT_CANDIDATE_TRANSFORMS)
                obs.update(position_tool_mm=transformed["position_tool_mm"],
                           position_camera_mm=result["position_camera_mm"],
                           position_3sigma_mm=result["position_3sigma_mm"],
                           center_disagreement_mm=result["center_disagreement_mm"],
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
