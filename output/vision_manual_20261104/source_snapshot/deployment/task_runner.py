"""Adapters for the existing four tasks. One owner caches at most one KPU model."""
try:
    import ujson as json
except ImportError:
    import json
import protocol as p
import runtime_config as cfg
from tasks.task_qr import QRTask
from tasks.task_bomb import BombTask
from tasks.task_aim import AimTask
from tasks.task_rescue import RescueTask
from tasks.task_rescue_object import RescueObjectTask
from yolo_detector import YoloDetector
from rescue_detector import RescueYoloDetector
from rescue_reference import RescueLayout,NativeRescueTableReference


def terminal_reason(reason):
    # Preserve the original message for display, but inspect nested detector
    # failures so an actual model/shape contract error is reported immediately.
    for _ in range(3):
        prefix=next((item for item in ("DETECTION_ERROR:","TARGET_DETECTOR_ERROR:","QR_DETECTION_ERROR:")
                     if reason.startswith(item)),None)
        if prefix is None:break
        reason=reason[len(prefix):]
    if reason in ("IMAGE_SIZE_MISMATCH","PREPROCESS_RAW_SIZE_MISMATCH","RAW_GEOMETRY_NOT_CONFIRMED"):
        return True
    prefixes=("MODEL_","TEST_BACKEND","HANDEYE_NOT_","CALIBRATION_","INVALID_",
              "TABLE_DIMENSIONS_NOT_","SLOT_LAYOUT_NOT_","SLOT_AXIS_","TABLE_TOP_ROI_NOT_",
              "TABLE_CORNER_ORDER_NOT_","INVALID_TABLE_","INVALID_SLOT_","SLOT_CENTER_OUTSIDE_",
              "TARGET_SLOT_NOT_KNOWN","SLOT_MAPPING_CONFLICT","SCAN_ORDER_NOT_","SCAN_BOUNDARIES_NOT_",
              "SCAN_MULTIPLE_","SCAN_TRANSITION_","SCAN_REVISIT_","SCAN_ORDER_AMBIGUOUS_",
              "TOO_MANY_TABLE_","RESCUE_TASK_FRAME_BUDGET_","TARGET_TASK_FRAME_BUDGET_")
    return any(reason.startswith(prefix) for prefix in prefixes)


class VisionJob:
    def __init__(self,owner,request,task):
        self.owner,self.request,self.task=owner,request,task
        self.cmd=request["cmd"]
        self.is_scan=self.cmd in p.SCAN_COMMANDS
        self.debug_trace=None
        self.debug_misses=0

    def enable_debug(self):
        if self.debug_trace is not None:return
        from vision_debug import DetectionTrace,DetectorProbe,QRPoseProbe
        kind={p.QR_START:"qr",p.BOMB_MEASURE:"bomb",p.AIM_MEASURE:"aim",p.RESCUE_MEASURE:"rescue"}[self.cmd]
        self.debug_trace=DetectionTrace(kind,self.request.get("target_id"))
        if self.cmd==p.QR_START:
            for name in ("estimator","high_resolution_estimator"):
                estimator=getattr(self.task,name,None)
                if estimator is not None:setattr(self.task,name,QRPoseProbe(estimator,self.debug_trace))
        else:
            self.task.detector=DetectorProbe(self.task.detector,self.debug_trace)

    def debug_snapshot(self):
        return self.debug_trace.snapshot() if self.debug_trace is not None else None

    def _debug_begin(self,frame):
        if self.debug_trace is not None:
            from vision_debug import image_size
            raw=frame.get("qr_image") if self.cmd==p.QR_START else None
            if raw is None:raw=frame["image"]
            self.debug_trace.begin(frame["frame_id"],image_size(raw))

    def preview(self,frame):
        """Detection-only feedback while waiting between reports; no pose samples."""
        if self.debug_trace is None:return
        self._debug_begin(frame)
        gray=enhanced=None
        try:
            if self.cmd==p.QR_START:
                gray=frame.get("qr_image")
                if gray is None:gray=frame["image"].to_grayscale()
                codes=gray.find_qrcodes()
                if codes:self.debug_misses=0
                else:self.debug_misses+=1
                if (not codes and getattr(self.task,"contrast_retry",False) and
                        self.debug_misses>=self.task.contrast_retry_every):
                    self.debug_misses=0
                    enhanced=gray.copy();enhanced.histeq(adaptive=False)
                    codes=enhanced.find_qrcodes()
                self.debug_trace.qr_result([{"corners_px":code.corners(),"payload":code.payload()} for code in codes])
            else:
                self.task.detector.detect(frame["image"])
        except Exception as exc:
            self.debug_trace.error="PREVIEW_ERROR:"+str(exc)
        finally:gray=enhanced=None

    def poll(self,frame):
        self._debug_begin(frame)
        raw,frame_id=frame["image"],frame["frame_id"]
        geometry=frame.get("geometry_confirmed",False)
        if self.cmd==p.QR_START:
            gray=frame.get("qr_image")
            if gray is None:gray=raw.to_grayscale()
            else:geometry=frame.get("qr_geometry_confirmed",False)
            try: result=self.task.measure(gray,frame_id,geometry_confirmed=geometry,stable=True)
            finally: gray=None
        else:
            if not self.is_scan and self.task.mode=="DONE":
                self.owner._start_measure(self.task,self.request)
            result=self.task.poll(raw,frame_id,geometry_confirmed=geometry,stable=not self.is_scan)
        result=dict(result)
        result["target_id"]=self.request.get("target_id")
        result["slot_id"]=result.get("slot_id",self.request.get("slot_id"))
        if self.cmd==p.QR_START: result.setdefault("reference_point","qr_center")
        elif self.cmd==p.BOMB_MEASURE: result.setdefault("reference_point","sphere_center")
        if self.is_scan and result.get("complete"):
            result["slot_mapping"]=result.get("color_to_slot",result.get("shape_to_slot"))
            result["sample_count"]=0
        return result

    def reset_window(self):
        if hasattr(self.task,"samples"): self.task.samples=[]
        method=getattr(self.task,"reset_window",None)
        if method is not None:method()

    def pause_window(self):
        self.reset_window()
        scan=getattr(self.task,"scan",None)
        scan=getattr(scan,"session",scan)  # Rescue wraps the same sequence tracker.
        if scan is not None:
            # Preserve confirmed order and the current latched identity. Partial
            # hits/releases must be observed again after resuming.
            scan.candidate,scan.hits,scan.center=None,0,None
            scan.blank_frames=0

    def cancel(self):
        if hasattr(self.task,"cancel"): self.task.cancel()


class VisionTaskFactory:
    def __init__(self,calibration,project_root,backend_factory=None):
        self.calibration,self.root=calibration,project_root.rstrip("/\\")
        self.backend_factory=backend_factory or cfg.create_model_backend
        self.detector,self.detector_kind=None,None
        self.selection=None
        self.maps={"bomb":None,"aim":None,"rescue":None}
        self.reference,self.layout=None,None

    def _json(self,relative):
        try:
            with open(self.root+"/"+relative,"r") as stream: return json.load(stream)
        except (OSError,ValueError): raise ValueError("MODEL_CONFIGURATION_ERROR" if "three_" in relative else "TABLE_CONFIGURATION_ERROR")

    def _model(self,kind):
        if self.detector_kind==kind and self.detector is not None and self.detector.ready: return self.detector
        if self.detector is not None: self.detector.close()
        self.detector,self.detector_kind=None,None
        filename="models/bomb_three_colors.template.json" if kind=="bomb" else "models/rescue_three_shapes.template.json"
        metadata=self._json(filename)
        if metadata.get("model_path") and not metadata["model_path"].startswith("/"):
            metadata["model_path"]=self.root+"/"+metadata["model_path"]
        backend=self.backend_factory(kind,metadata) if metadata.get("model_path") else None
        detector=(YoloDetector(metadata,backend) if kind=="bomb" else RescueYoloDetector(metadata,backend))
        loaded=detector.load()
        if not loaded["ok"]: raise ValueError(loaded["reason"])
        self.detector,self.detector_kind=detector,kind
        return detector

    def _rescue_reference(self):
        if self.reference is None:
            self.layout=RescueLayout(self._json("models/rescue_table.template.json"))
            self.reference=NativeRescueTableReference(self.calibration,self.layout)
        return self.reference

    def _kind(self,cmd):
        if cmd in (p.BOMB_SCAN,p.BOMB_MEASURE): return "bomb"
        if cmd in (p.AIM_SCAN,p.AIM_MEASURE): return "aim"
        if cmd in (p.RESCUE_SCAN,p.RESCUE_MEASURE): return "rescue"
        return None

    def _start_measure(self,task,request):
        cmd,target,slot=request["cmd"],request.get("target_id"),request.get("slot_id")
        if cmd==p.BOMB_MEASURE: task.start_refine(target,slot)
        else: task.start_measure(target,slot,max_frames=1000000)

    def start(self,request):
        cmd=request["cmd"]
        if cmd==p.QR_START:
            self.selection=None
            self.maps={"bomb":None,"aim":None,"rescue":None}
            task=QRTask(self.calibration,high_resolution_size=cfg.QR_CAPTURE_SIZE if cfg.QR_HIGH_RES_ENABLED else None,
                        contrast_retry=cfg.QR_CONTRAST_RETRY_ENABLED,
                        contrast_retry_every=cfg.QR_CONTRAST_RETRY_EVERY_N_FRAMES)
            return VisionJob(self,request,task)
        kind=self._kind(cmd)
        if request.get("target_id") is None:
            key={"bomb":"bomb_color_id","aim":"target_color_id","rescue":"rescue_shape_id"}[kind]
            if self.selection is None: raise ValueError("TARGET_NOT_SELECTED")
            request["target_id"]=self.selection[key]
        mapping=self.maps[kind]
        if cmd in p.SCAN_COMMANDS:
            self.maps[kind]=None
            if not cfg.SCAN_ORDER_CONFIRMED or not cfg.SCAN_START_BOUNDARY_CONFIRMED:
                raise ValueError("SCAN_NOT_CONFIGURED")
        else:
            selected=mapping.get(request["target_id"]) if mapping is not None else None
            if request.get("slot_id") is None: request["slot_id"]=selected
            elif selected is not None and selected!=request["slot_id"]: raise ValueError("SLOT_MAPPING_CONFLICT")
            if cmd==p.RESCUE_MEASURE and cfg.RESCUE_REFERENCE_MODE=="table" and request.get("slot_id") is None: raise ValueError("TARGET_SLOT_NOT_KNOWN")
        if kind=="bomb": task=BombTask(self.calibration,self._model("bomb"))
        elif kind=="aim": task=AimTask(self.calibration,measure_hits=1)
        else:
            detector=self._model("rescue")
            if cfg.RESCUE_REFERENCE_MODE=="object":
                task=RescueObjectTask(self.calibration,detector)
            else:
                reference=self._rescue_reference()
                if cmd==p.RESCUE_MEASURE and reference.readiness()!="OK": raise ValueError(reference.readiness())
                task=RescueTask(self.calibration,detector,self.layout,reference,measure_hits=1)
        if cmd in p.SCAN_COMMANDS:
            if kind=="bomb": task.start_scan(request["target_id"],True,True)
            else: task.start_scan(request["target_id"],True,True,max_frames=1000000)
            tracker=getattr(task.scan,"session",task.scan)
            tracker.require_release=cfg.SCAN_REQUIRE_BLANK_GAP
            tracker.ambiguous_limit=cfg.SCAN_MAX_AMBIGUOUS_FRAMES
        else: self._start_measure(task,request)
        return VisionJob(self,request,task)

    def completed(self,request,observation):
        cmd=request["cmd"]
        if cmd==p.QR_START: self.selection=observation["task"]
        elif cmd in p.SCAN_COMMANDS: self.maps[self._kind(cmd)]=dict(observation["slot_mapping"])

    def reset_session(self):
        self.selection=None
        self.maps={"bomb":None,"aim":None,"rescue":None}

    def close(self):
        try:
            if self.detector is not None: self.detector.close()
        finally:
            self.detector,self.detector_kind=None,None
            if self.reference is not None: self.reference.close()
