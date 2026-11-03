"""Load existing calibration files. Never promote a candidate to accepted."""
try:
    import ujson as json
except ImportError:
    import json
from geometry import CameraModel, transform_point, validate_rigid


class Calibration:
    def __init__(self, K, D, T_tool_from_camera=None, handeye_accepted=False,
                 image_size=(640, 480), quality_review=None):
        self.camera = CameraModel(K, D, image_size)
        self.T_tool_from_camera = (validate_rigid(T_tool_from_camera)
                                   if T_tool_from_camera is not None else None)
        self.handeye_accepted = handeye_accepted is True
        self.quality_review = quality_review or {}
        # E is the existing calibration's commanded TCP / gripper-center frame.
        self.tool_origin = "commanded_gripper_center_tcp"

    def point_to_tool(self, point_camera_mm, allow_candidate=True):
        if self.T_tool_from_camera is None:
            return {"ok": False, "reason": "HANDEYE_NOT_CONFIGURED", "position_tool_mm": None,
                    "control_ready": False}
        if not self.handeye_accepted and not allow_candidate:
            return {"ok": False, "reason": "HANDEYE_NOT_ACCEPTED", "position_tool_mm": None,
                    "control_ready": False}
        return {"ok": True, "reason": "OK" if self.handeye_accepted else "HANDEYE_CANDIDATE_ONLY",
                "position_tool_mm": transform_point(self.T_tool_from_camera, point_camera_mm),
                "handeye_accepted": self.handeye_accepted, "control_ready": self.handeye_accepted,
                "coordinate_frame": "E_commanded_gripper_center", "units": "mm"}


def load_calibration(project_root="/sdcard", handeye_accepted_override=False):
    root = project_root.rstrip("/\\")
    with open(root+"/calibration/results/camera_params.json", "r") as stream:
        camera = json.load(stream)
    handeye = None
    review = {}
    try:
        with open(root+"/calibration/handeye/session_20261025_fixed/result/handeye_result.json", "r") as stream:
            handeye_data = json.load(stream)
        handeye = handeye_data["T_tool_from_camera"]
        review = handeye_data.get("quality_review", {})
    except OSError:
        pass  # Camera-only observations remain usable; no identity fallback.
    return Calibration(camera["K"], camera["D"], handeye,
                       review.get("recommended_for_robot_use", False) or (handeye_accepted_override is True and handeye is not None),
                       camera["image_size"], review)
