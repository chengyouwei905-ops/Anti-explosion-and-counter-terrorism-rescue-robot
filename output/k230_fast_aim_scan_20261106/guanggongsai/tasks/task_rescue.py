from boundary_policy import enabled as boundary_checks
"""Rescue shape scan + known table-slot observation in the gripper frame.

No communication, vehicle pose fusion, arm motion or grab execution.
"""
import rescue_config as settings
from config import IMAGE_SIZE, GEOMETRY_ID, REPORT_CANDIDATE_TRANSFORMS
from geometry import finite, norm, transform_point
from rescue_geometry import project_table_point
from tasks.task_bomb import BombScanSession


def valid_shape(shape_id):
    return type(shape_id) is int and 1 <= shape_id <= 3


def _error(reason,frame_id=None):
    return {"ok":False,"reason":reason,"frame_id":frame_id,
            "position_camera_mm":None,"position_tool_mm":None,
            "grasp_position_camera_mm":None,"grasp_position_tool_mm":None,
            "orientation_valid":False,"control_ready":False}


class RescueScanSession:
    """Reuse three-identity scan rules; color names never leak into rescue output."""
    def __init__(self,target_shape_id,scan_order_confirmed=False,start_boundary_confirmed=False,gate=None):
        if not valid_shape(target_shape_id): raise ValueError("INVALID_SHAPE_ID")
        self.session = BombScanSession(target_shape_id,scan_order_confirmed,start_boundary_confirmed,
                                       gate or settings.SCAN_GATE_RAW_PX,
                                       settings.SCAN_MIN_HITS,settings.SCAN_RELEASE_FRAMES)

    def _rename(self,result):
        return {"ok":result["ok"],"reason":result["reason"],"complete":result["complete"],
                "observed_shape_order":result["observed_order"],
                "shape_to_slot":result["color_to_slot"],"target_slot":result["target_slot"]}

    def update(self,detections,frame_id):
        converted=[]
        for item in detections:
            if not valid_shape(item["shape_id"]):
                self.session.failed=True
                return self._rename(self.session._state("INVALID_SHAPE_ID"))
            converted.append({"color_id":item["shape_id"],"bbox_raw_px":item["bbox_raw_px"]})
        return self._rename(self.session.update(converted,frame_id))

    def drop_frame(self,reason):
        self.session.drop_frame(reason)

    def fail(self):
        self.session.drop_frame("RESCUE_DETECTION_FRAME_INVALID")

    def finish(self):
        return self._rename(self.session.finish())


def measure_rescue_reference(calibration,layout,table_pose,shape_id,slot_id,bbox_raw_px,
                              frame_id,geometry_confirmed=False,stable=False,
                              source="external_detections"):
    """A table-plane slot center is a fixed landmark, not a bbox/object center."""
    if not valid_shape(shape_id): return _error("INVALID_SHAPE_ID",frame_id)
    if not geometry_confirmed: return _error("RAW_GEOMETRY_NOT_CONFIRMED",frame_id)
    if not stable: return _error("OBSERVATION_NOT_STABLE",frame_id)
    reason=layout.ready_reason()
    if reason != "OK": return _error(reason,frame_id)
    if not table_pose.get("ok"): return _error(table_pose.get("reason","TABLE_REFERENCE_NOT_READY"),frame_id)
    if table_pose.get("frame_id") != frame_id: return _error("TABLE_FRAME_MISMATCH",frame_id)
    if any(abs(table_pose.get(key,0)-layout.data[key]) > 1e-6
           for key in ("table_width_mm","table_depth_mm")):
        return _error("TABLE_GEOMETRY_MISMATCH",frame_id)
    try:
        if len(bbox_raw_px) != 4 or not all(finite(v) for v in bbox_raw_px) or min(bbox_raw_px[2:]) <= 0:
            raise ValueError("INVALID_DETECTION_BOX")
        x,y,w,h=bbox_raw_px
        if boundary_checks() and (x < 0 or y < 0 or x+w > IMAGE_SIZE[0] or y+h > IMAGE_SIZE[1]):
            raise ValueError("RESCUE_TARGET_CLIPPED")
        table_point=layout.slot_point(slot_id)
        measured=project_table_point(calibration.camera,table_pose,table_point)
        margin=settings.SLOT_ASSOCIATION_MARGIN_PX
        matching=[]
        for slot in (1,2,3):
            p=transform_point(table_pose["T_camera_from_table"],layout.slot_point(slot))
            u,v=calibration.camera.project_ray(p)
            if x-margin <= u <= x+w+margin and y-margin <= v <= y+h+margin:
                matching.append(slot)
        if matching != [slot_id]:
            raise ValueError("SLOT_TARGET_ASSOCIATION_FAILED")
        transformed=calibration.point_to_tool(measured["position_camera_mm"],REPORT_CANDIDATE_TRANSFORMS)
        result={"ok":transformed["ok"],"reason":transformed["reason"],"frame_id":frame_id,
                "shape_id":shape_id,"shape":settings.SHAPE_NAMES[shape_id],"slot_id":slot_id,
                "position_camera_mm":measured["position_camera_mm"],
                "position_tool_mm":transformed["position_tool_mm"],
                "reference_point":"table_slot_center_at_table_surface","reference_is_fixed":True,
                "coordinate_frame":"E_commanded_gripper_center","units":"mm",
                "orientation_valid":False,"handeye_accepted":calibration.handeye_accepted,
                "geometry_valid":True,"geometry_id":GEOMETRY_ID,"source":source,
                "control_ready":transformed["control_ready"] and source=="yolo_rescue"
                                and table_pose.get("source") in ("native_table_rectangle","verified_table_features"),
                "T_camera_from_table":table_pose["T_camera_from_table"],
                "table_pose_frame_id":frame_id,"quality":dict(table_pose["quality"]),
                "grasp_position_camera_mm":None,"grasp_position_tool_mm":None,
                "grasp_reference_point":None,"grasp_reason":"GRASP_HEIGHT_NOT_CONFIGURED"}
        result["quality"]["reference_position_3sigma_mm"]=measured["position_3sigma_mm"]
        try:
            grasp_point=layout.grasp_point(slot_id)
            if grasp_point is not None:
                grasp=project_table_point(calibration.camera,table_pose,grasp_point)
                converted=calibration.point_to_tool(grasp["position_camera_mm"],REPORT_CANDIDATE_TRANSFORMS)
                result.update(grasp_position_camera_mm=grasp["position_camera_mm"],
                              grasp_position_tool_mm=converted["position_tool_mm"],
                              grasp_reference_point="configured_height_on_slot_axis",
                              grasp_reason=converted["reason"])
        except (ValueError,KeyError) as exc:
            result["grasp_reason"]=str(exc)
        return result
    except (ValueError,TypeError,IndexError,KeyError,ZeroDivisionError) as exc:
        return _error(str(exc),frame_id)


class RescueTask:
    def __init__(self,calibration,detector,layout,table_reference=None,
                 measure_hits=settings.MEASURE_MIN_HITS,max_scatter_mm=settings.MAX_SCATTER_MM):
        if type(measure_hits) is not int or not 1 <= measure_hits <= 10 or not finite(max_scatter_mm) or max_scatter_mm <= 0:
            raise ValueError("INVALID_MEASUREMENT_STABILITY")
        self.calibration,self.detector,self.layout=calibration,detector,layout
        self.table_reference=table_reference
        self.measure_hits,self.max_scatter_mm=measure_hits,max_scatter_mm
        self.mode,self.scan,self.mapping,self.samples="IDLE",None,None,[]
        self.last_frame,self.frames,self.max_frames=-1,0,0
        self.target_shape,self.slot_id=None,None

    def _budget(self,max_frames):
        if type(max_frames) is not int or max_frames < 1: raise ValueError("INVALID_FRAME_BUDGET")
        self.max_frames=max_frames

    def start_scan(self,target_shape_id,scan_order_confirmed=False,start_boundary_confirmed=False,
                   gate=None,max_frames=2400):
        self._budget(max_frames)
        self.scan=RescueScanSession(target_shape_id,scan_order_confirmed,start_boundary_confirmed,gate)
        self.mode,self.mapping,self.samples="SCAN",None,[]
        self.last_frame,self.frames=-1,0

    def start_measure(self,shape_id,slot_id=None,max_frames=180):
        if not valid_shape(shape_id): raise ValueError("INVALID_SHAPE_ID")
        if slot_id is None and self.mapping is not None: slot_id=self.mapping.get(shape_id)
        if not valid_shape(slot_id): raise ValueError("TARGET_SLOT_NOT_KNOWN")
        if self.mapping is not None and self.mapping.get(shape_id) != slot_id:
            raise ValueError("SLOT_MAPPING_CONFLICT")
        self._budget(max_frames)
        self.target_shape,self.slot_id=shape_id,slot_id
        self.mode,self.scan,self.samples="MEASURE",None,[]
        self.last_frame,self.frames=-1,0

    def _invalid(self,reason,frame_id):
        self.samples=[]
        return _error(reason,frame_id)

    def poll(self,raw,frame_id,geometry_confirmed=False,stable=False):
        if self.mode not in ("SCAN","MEASURE"): return _error("TASK_NOT_ACTIVE",frame_id)
        if type(frame_id) is not int or frame_id <= self.last_frame:
            return _error("STALE_OR_DUPLICATE_FRAME",frame_id)
        previous=self.last_frame
        self.last_frame,self.frames=frame_id,self.frames+1
        if self.frames > self.max_frames:
            self.mode="ERROR"
            return self._invalid("RESCUE_TASK_FRAME_BUDGET_EXCEEDED",frame_id)
        if not geometry_confirmed or (raw.width(),raw.height()) != IMAGE_SIZE:
            if self.mode=="SCAN": self.scan.fail()
            return self._invalid("RAW_GEOMETRY_NOT_CONFIRMED",frame_id)
        if self.mode=="MEASURE" and not stable:
            return self._invalid("OBSERVATION_NOT_STABLE",frame_id)
        try:
            detection=self.detector.detect(raw)
            if not detection["ok"]:
                if self.mode=="SCAN": self.scan.fail()
                return self._invalid(detection["reason"],frame_id)
            if self.mode=="SCAN":
                result=self.scan.update(detection["detections"],frame_id)
                result.update(frame_id=frame_id,task="rescue")
                if result["complete"]:
                    self.mapping=dict(result["shape_to_slot"])
                    self.mode="DONE"
                return result
            matches=[d for d in detection["detections"] if d["shape_id"]==self.target_shape]
            if not matches: return self._invalid("RESCUE_TARGET_NOT_FOUND",frame_id)
            if len(matches)!=1: return self._invalid("MULTIPLE_SAME_SHAPE_TARGETS",frame_id)
            if boundary_checks() and matches[0].get("bbox_clipped",False): return self._invalid("RESCUE_TARGET_CLIPPED",frame_id)
            if self.table_reference is None: return self._invalid("TABLE_REFERENCE_NOT_READY",frame_id)
            pose=self.table_reference.observe(raw,frame_id,geometry_confirmed=True)
            result=measure_rescue_reference(self.calibration,self.layout,pose,self.target_shape,
                       self.slot_id,matches[0]["bbox_raw_px"],frame_id,True,True,detection.get("source","unknown"))
        except Exception as exc:
            if self.mode=="SCAN": self.scan.fail()
            return self._invalid("RESCUE_OBSERVATION_ERROR:"+str(exc),frame_id)
        if not result["ok"]:
            self.samples=[]
            return result
        if previous >= 0 and frame_id != previous+1: self.samples=[]
        self.samples.append((frame_id,result["position_camera_mm"],result["grasp_position_camera_mm"]))
        self.samples=self.samples[-self.measure_hits:]
        mean=[sum(s[1][i] for s in self.samples)/len(self.samples) for i in range(3)]
        scatter=max(norm([s[1][i]-mean[i] for i in range(3)]) for s in self.samples)
        if scatter > self.max_scatter_mm:
            self.samples=self.samples[-1:]
            return _error("RESCUE_POSITION_NOT_STABLE",frame_id)
        if len(self.samples)<self.measure_hits:
            return _error("RESCUE_MEASUREMENT_STABILIZING",frame_id)
        result["position_camera_mm"]=mean
        result["position_tool_mm"]=self.calibration.point_to_tool(mean,REPORT_CANDIDATE_TRANSFORMS)["position_tool_mm"]
        if all(s[2] is not None for s in self.samples):
            grasp_mean=[sum(s[2][i] for s in self.samples)/len(self.samples) for i in range(3)]
            grasp_scatter=max(norm([s[2][i]-grasp_mean[i] for i in range(3)]) for s in self.samples)
            if grasp_scatter <= self.max_scatter_mm:
                result["grasp_position_camera_mm"]=grasp_mean
                result["grasp_position_tool_mm"]=self.calibration.point_to_tool(grasp_mean,REPORT_CANDIDATE_TRANSFORMS)["position_tool_mm"]
                result["grasp_scatter_mm"]=grasp_scatter
            else:
                result.update(grasp_position_camera_mm=None,grasp_position_tool_mm=None,
                              grasp_reason="GRASP_POSITION_NOT_STABLE")
        elif any(s[2] is not None for s in self.samples):
            result.update(grasp_position_camera_mm=None,grasp_position_tool_mm=None,
                          grasp_reason="GRASP_MEASUREMENT_NOT_STABLE")
        result.update(sample_frame_ids=[s[0] for s in self.samples],measurement_scatter_mm=scatter,
                      measurement_complete=True)
        self.samples,self.mode=[],"DONE"
        return result

    def finish_scan(self):
        if self.scan is None or self.mode not in ("SCAN","DONE"): return _error("SCAN_NOT_ACTIVE")
        self.mode="DONE"
        return self.scan.finish()

    def invalidate_mapping(self):
        self.mapping=None
        self.cancel()

    def cancel(self):
        self.mode,self.scan,self.samples="CANCELLED",None,[]

    def close(self):
        self.cancel()
        try:
            self.detector.close()
        finally:
            if self.table_reference is not None: self.table_reference.close()
