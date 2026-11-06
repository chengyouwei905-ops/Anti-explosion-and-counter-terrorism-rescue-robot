"""Rescue scan and target-axis-midpoint measurement, independent of table view."""
from tasks.task_rescue import RescueScanSession,valid_shape,_error
from rescue_object_pose import RescueObjectEstimator
from config import IMAGE_SIZE,REPORT_CANDIDATE_TRANSFORMS
import rescue_config as cfg


class RescueObjectTask:
    def __init__(self,calibration,detector,estimator=None):
        self.calibration,self.detector=calibration,detector
        self.estimator=estimator or RescueObjectEstimator(calibration.camera)
        self.mode="IDLE";self.scan=None;self.samples=[]
        self.target_shape=self.slot_id=None
        self.last_frame=-1;self.frames=0;self.max_frames=1000000
    def start_scan(self,target_shape_id,scan_order_confirmed=True,start_boundary_confirmed=True,gate=None,max_frames=1000000):
        self.scan=RescueScanSession(target_shape_id,scan_order_confirmed,start_boundary_confirmed,gate)
        self.mode="SCAN";self.last_frame=-1;self.frames=0;self.max_frames=max_frames
    def start_measure(self,shape_id,slot_id=None,max_frames=1000000):
        if not valid_shape(shape_id) or (slot_id is not None and not valid_shape(slot_id)):raise ValueError("INVALID_TARGET")
        self.target_shape,self.slot_id=shape_id,slot_id
        self.mode="MEASURE";self.last_frame=-1;self.frames=0;self.max_frames=max_frames
    def poll(self,raw,frame_id,geometry_confirmed=False,stable=False):
        if self.mode not in ("SCAN","MEASURE"):return _error("TASK_NOT_ACTIVE",frame_id)
        if type(frame_id) is not int or frame_id<=self.last_frame:return _error("STALE_OR_DUPLICATE_FRAME",frame_id)
        self.last_frame=frame_id;self.frames+=1
        if self.frames>self.max_frames:return _error("RESCUE_TASK_FRAME_BUDGET_EXCEEDED",frame_id)
        if not geometry_confirmed or (raw.width(),raw.height())!=IMAGE_SIZE:return _error("RAW_GEOMETRY_NOT_CONFIRMED",frame_id)
        detections=self.detector.detect(raw)
        if not detections["ok"]:
            if self.mode=="SCAN":self.scan.drop_frame(detections["reason"])
            return _error(detections["reason"],frame_id)
        if self.mode=="SCAN":
            result=self.scan.update(detections["detections"],frame_id)
            result.update(frame_id=frame_id,task="rescue")
            if result.get("complete"):self.mode="DONE"
            return result
        if not stable:return _error("OBSERVATION_NOT_STABLE",frame_id)
        matches=[d for d in detections["detections"] if d["shape_id"]==self.target_shape]
        if not matches:return _error("RESCUE_TARGET_NOT_FOUND",frame_id)
        if len(matches)!=1:return _error("MULTIPLE_SAME_SHAPE_TARGETS",frame_id)
        try:
            measured=self.estimator.fit_bbox_width(matches[0]["bbox_raw_px"],self.target_shape)
        except (ValueError,RuntimeError,OSError) as exc:return _error(str(exc),frame_id)
        if not measured["ok"]:return _error(measured["reason"],frame_id)
        transformed=self.calibration.point_to_tool(measured["position_camera_mm"],REPORT_CANDIDATE_TRANSFORMS)
        measured.update(frame_id=frame_id,shape_id=self.target_shape,shape=cfg.SHAPE_NAMES[self.target_shape],
                        slot_id=self.slot_id,position_tool_mm=transformed["position_tool_mm"],
                        coordinate_frame="E_commanded_gripper_center",units="mm",
                        handeye_accepted=self.calibration.handeye_accepted,
                        control_ready=transformed["control_ready"] and detections.get("source")=="yolo_rescue",
                        ok=transformed["ok"],reason=transformed["reason"],source=detections.get("source"),
                        geometry_valid=True,reference_is_fixed=False,
                        grasp_position_tool_mm=None,grasp_position_camera_mm=None)
        self.mode="DONE"
        return measured
    def reset_window(self):self.samples=[];self.estimator.reset()
    def cancel(self):self.mode="CANCELLED";self.scan=None;self.reset_window()
