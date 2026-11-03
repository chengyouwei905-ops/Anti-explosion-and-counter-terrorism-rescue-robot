"""K230 local state machine; frame work and result transmission stay in main."""
import protocol as p
import runtime_config as cfg
from measurement import MeasurementAverage,QRMeasurementAverage
from camera_service import ticks_ms,ticks_diff
from task_runner import terminal_reason

IDLE,SETTLING,RUNNING,REPORT="IDLE","SETTLING","RUNNING","REPORT"
PAUSED="PAUSED"


class TaskManager:
    def __init__(self,link,factory,clock=ticks_ms,diff=ticks_diff):
        self.link,self.factory,self.clock,self.diff=link,factory,clock,diff
        self.state=IDLE
        self.request=self.job=self.average=self.pending=None
        self.history=[]
        self.counter=self.run_id=0
        self.started_ms=self.timeout_ms=0
        self.epoch=None
        self.last_frame_id=None
        self.last_reason="IDLE"
        self.last_report=None
        self.last_scan_error=None
        self.last_scan_order=[]
        self.transmit_error=None
        self.report_attempts=0
        self.paused=False
        self._elapsed_before_pause=0
        self._settle_started_ms=0
        self.last_measurement_diagnostic=None
        self._reset_measurement_diagnostics()

    def _reset_measurement_diagnostics(self):
        self.measurement_frames=0
        self.measurement_peak_samples=0
        self.measurement_rejections={}
        self.last_measurement_error=None
        self._measurement_log_ms=None

    def _measurement_snapshot(self):
        if self.request is None or self.request["cmd"] in p.SCAN_COMMANDS:return None
        return {"cmd":self.request["cmd"],"input_seq":self.request["input_seq"],
                "target_id":self.request.get("target_id"),"slot_id":self.request.get("slot_id"),
                "elapsed_ms":self._elapsed(),"frames":self.measurement_frames,
                "samples":len(self.average.samples) if self.average is not None else 0,
                "peak_samples":self.measurement_peak_samples,"last_state":self.last_reason,
                "last_error":self.last_measurement_error,
                "rejections":dict(self.measurement_rejections)}

    def _record_measurement_reason(self,reason):
        count=len(self.average.samples) if self.average is not None else 0
        self.measurement_peak_samples=max(self.measurement_peak_samples,count)
        if reason not in ("OK","MEASUREMENT_STABILIZING"):
            self.last_measurement_error=reason
            # Bounded diagnostic storage, including unexpected exception strings.
            key=reason if reason in self.measurement_rejections or len(self.measurement_rejections)<8 else "OTHER"
            self.measurement_rejections[key]=self.measurement_rejections.get(key,0)+1

    def log_measurement_state(self):
        if self.state not in (SETTLING,RUNNING):return
        diagnostic=self._measurement_snapshot()
        if diagnostic is None:return
        now=self.clock()
        if self._measurement_log_ms is not None and self.diff(now,self._measurement_log_ms)<1000:return
        self._measurement_log_ms=now
        print("K230_MEASURE_STATE",p.NAMES.get(diagnostic["cmd"],diagnostic["cmd"]),
              "seq",diagnostic["input_seq"],"target",diagnostic["target_id"],"slot",diagnostic["slot_id"],
              "elapsed_ms",diagnostic["elapsed_ms"],"reason",diagnostic["last_state"],
              "samples",diagnostic["samples"],"peak",diagnostic["peak_samples"],
              "frames",diagnostic["frames"],"last_error",diagnostic["last_error"],
              "rejections",diagnostic["rejections"])

    def pause(self):
        """Freeze the active request; never transmit a cancellation/old pose here."""
        if self.paused: return
        self.paused=True
        self.state=PAUSED
        if self.request is not None:
            self._elapsed_before_pause+=self.diff(self.clock(),self.started_ms)
            if self.average is not None: self.average.reset()
            if self.job is not None:
                method=getattr(self.job,"pause_window",self.job.reset_window)
                method()
            # Scan results contain identities/slots, not a stale measured pose.
            if (self.pending is not None and self.pending[0] in (p.SUCCESS,p.OBSERVATION_ONLY)
                    and self.job is not None and not self.job.is_scan):
                self.pending=None

    def reset_session(self):
        # Total STOP mirrors MCU -> TASK_IDLE; no stale terminal result is sent.
        self._discard_active()
        self.paused=True
        self.state=PAUSED
        self.history=[]
        self._elapsed_before_pause=0
        reset=getattr(self.factory,"reset_session",None)
        if reset is not None:reset()

    def resume(self):
        if not self.paused: return
        self.paused=False
        self.started_ms=self._settle_started_ms=self.clock()
        if self.request is None:
            self.state=IDLE
        elif self.pending is not None:
            self.state=REPORT
        else:
            self.state=RUNNING if self.job is not None and self.job.is_scan else SETTLING

    def _elapsed(self):
        return self._elapsed_before_pause+self.diff(self.clock(),self.started_ms)

    @property
    def active_task(self):
        return p.NAMES.get(self.request["cmd"]) if self.request is not None else None

    def _new_run_id(self):
        self.counter=(self.counter+1)&0xffff
        return self.counter

    def _seen(self,key,now):
        self.history=[(k,t) for k,t in self.history if self.diff(now,t)<cfg.REQUEST_DEDUP_MS]
        if self.request is not None and self.request["key"]==key: return True
        return any(k==key for k,_ in self.history)

    def _remember(self,key,now):
        self.history.append((key,now))
        self.history=self.history[-cfg.REQUEST_HISTORY_SIZE:]

    def _transmit_once(self,request,run_id,status,reason,observation=None):
        self.report_attempts+=1
        self.last_report=None
        try:
            try:
                payload=p.encode_result(request,run_id,status,reason,observation)
            except Exception:
                status,reason,observation=p.FAILED,"VISION_ERROR",None
                payload=p.encode_result(request,run_id,status,reason)
            print("K230_RESULT",p.NAMES.get(request["cmd"],request["cmd"]),"seq",request["input_seq"],"status",status,"reason",reason)
            self.link.send(p.RESULT,payload)
            self.last_report={"run_id":run_id,"request":dict(request),"status":status,"reason":reason,
                              "observation":observation,"payload":payload}
        except Exception as exc:
            # A short/failed UART write is not retried: do not duplicate a terminal report.
            self.transmit_error=type(exc).__name__+":"+str(exc)
            print("VISION_TX_FAILED",self.transmit_error)
        self.last_reason=reason

    def _discard_active(self):
        try:
            if self.job is not None: self.job.cancel()
        except Exception as exc:
            self.last_reason="CANCEL_ERROR:"+str(exc)
        finally:
            self.state=IDLE
            self.request=self.job=self.average=self.pending=None
            self.epoch=None
            self.last_frame_id=None

    def _finish_active(self,status,reason,observation=None):
        request,run_id=self.request,self.run_id
        diagnostic=self._measurement_snapshot()
        if diagnostic is not None:
            diagnostic.update(status=status,reason=reason)
            self.last_measurement_diagnostic=diagnostic
            print("K230_MEASURE_END",p.NAMES.get(request["cmd"],request["cmd"]),
                  "seq",request["input_seq"],"status",status,"reason",reason,
                  "last_state",diagnostic["last_state"],"last_error",diagnostic["last_error"],
                  "peak",diagnostic["peak_samples"],"frames",diagnostic["frames"],
                  "elapsed_ms",diagnostic["elapsed_ms"],"rejections",diagnostic["rejections"])
        self._discard_active()
        if request is not None:
            self._transmit_once(request,run_id,status,reason,observation)
            if self.last_report is not None and diagnostic is not None:
                self.last_report["diagnostic"]=diagnostic

    def on_command(self,frame):
        if self.paused: return
        if frame.cmd==p.HEARTBEAT: return
        now=self.clock()
        key=(frame.seq,frame.cmd,bytes(frame.payload))
        if self._seen(key,now): return
        self._remember(key,now)
        try:
            request=p.decode_command(frame)
        except ValueError as exc:
            invalid={"cmd":frame.cmd,"input_seq":frame.seq,"target_id":None,"slot_id":None,"key":key}
            self._transmit_once(invalid,self._new_run_id(),p.FAILED,str(exc))
            return
        if request["cmd"]==p.STOP:
            if self.request is not None: self._finish_active(p.CANCELLED,"CANCELLED")
            return  # STOP while idle is intentionally silent.
        if self.request is not None: self._finish_active(p.CANCELLED,"PREEMPTED")
        self.request=request
        self._reset_measurement_diagnostics()
        self.last_measurement_diagnostic=None
        self.run_id=self._new_run_id()
        self.started_ms=now
        self._settle_started_ms=now
        self._elapsed_before_pause=0
        self.timeout_ms=cfg.SCAN_TIMEOUT_MS if request["cmd"] in p.SCAN_COMMANDS else cfg.MEASUREMENT_TIMEOUT_MS
        self.average=MeasurementAverage(cfg.MEASUREMENT_SAMPLES,cfg.MAX_POSITION_SCATTER_MM)
        if request["cmd"]==p.QR_START and cfg.QR_SAMPLE_WINDOW_MS>0:
            self.average=QRMeasurementAverage(cfg.MEASUREMENT_SAMPLES,cfg.MAX_POSITION_SCATTER_MM,
                                              cfg.QR_SAMPLE_WINDOW_MS,
                                              self.clock,self.diff)
        self.pending=None
        self.last_scan_error=None
        self.last_scan_order=[]
        self.last_reason="STARTING"
        try:
            self.job=self.factory.start(request)
            self.state=RUNNING if self.job.is_scan else SETTLING
        except Exception as exc:
            self.job=None
            self.pending=(p.FAILED,str(exc),None)
            self.state=REPORT

    def tick(self):
        if self.state not in (SETTLING,RUNNING): return
        elapsed=self._elapsed()
        if elapsed>=self.timeout_ms:
            self.pending=(p.FAILED,"TIMEOUT",None)
            self.state=REPORT
        elif self.state==SETTLING and self.diff(self.clock(),self._settle_started_ms)>=cfg.SETTLE_MS:
            self.state=RUNNING
        self.log_measurement_state()

    def observe_camera_epoch(self,epoch):
        if self.request is None: return
        if self.epoch is None:
            self.epoch=epoch
            return
        if self.epoch==epoch: return
        self.epoch=epoch
        if self.job is not None and self.job.is_scan and self.last_frame_id is not None:
            self.pending=(p.FAILED,"CAMERA_ERROR",None)
            self.state=REPORT
        else:
            if self.average is not None: self.average.reset()
            if self.job is not None: self.job.reset_window()

    def poll_frame(self,frame):
        if self.state!=RUNNING or self.job is None: return
        if self.last_frame_id is not None and frame["frame_id"]<=self.last_frame_id: return
        self.last_frame_id=frame["frame_id"]
        self.observe_camera_epoch(frame["epoch"])
        if self.state!=RUNNING: return
        if not self.job.is_scan:self.measurement_frames+=1
        try:
            result=self.job.poll(frame)
            if self._elapsed()>=self.timeout_ms:
                if not self.job.is_scan:
                    self.last_reason=result.get("reason","UNKNOWN_MEASUREMENT") if not result.get("ok") else "MEASUREMENT_AFTER_TIMEOUT"
                    self._record_measurement_reason(self.last_reason)
                self.pending=(p.FAILED,"TIMEOUT",None); self.state=REPORT; return
            if self.job.is_scan:
                reason=result.get("reason","SCANNING")
                order=result.get("observed_order",result.get("observed_shape_order"))
                if order is not None and list(order)!=self.last_scan_order:
                    self.last_scan_order=list(order)
                    print("K230_SCAN_PROGRESS",p.NAMES[self.request["cmd"]],"seq",self.request["input_seq"],"order",self.last_scan_order)
                ordinary=("OK","SCANNING","SCAN_WAIT_TARGET","SCAN_STABILIZING","SCAN_TARGET_RECORDED","SCAN_ALREADY_COUNTED")
                if reason not in ordinary:
                    if self.last_scan_error!=reason:
                        print("K230_SCAN_DIAGNOSTIC",p.NAMES[self.request["cmd"]],"seq",self.request["input_seq"],"reason",reason,"order",self.last_scan_order)
                    self.last_scan_error=reason
                if result.get("ok") and result.get("complete"):
                    self.pending=(p.SUCCESS,"OK",result)
                    self.state=REPORT
                elif terminal_reason(result.get("reason","")):
                    self.pending=(p.FAILED,result["reason"],None)
                    self.state=REPORT
                else: self.last_reason=result.get("reason","SCANNING")
                return
            if isinstance(self.average,QRMeasurementAverage):
                averaged,reason=self.average.add(result,frame.get("capture_return_ms"))
            else:
                averaged,reason=self.average.add(result)
            self.last_reason=reason
            self._record_measurement_reason(reason)
            if averaged is not None:
                accepted=averaged.get("control_ready") is True
                if not accepted and not cfg.ALLOW_CANDIDATE_REPORTS:
                    self.pending=(p.FAILED,"CALIBRATION_NOT_READY",None)
                else:
                    self.pending=(p.SUCCESS if accepted else p.OBSERVATION_ONLY,
                                  "OK" if accepted else "HANDEYE_CANDIDATE_ONLY",averaged)
                self.state=REPORT
            elif terminal_reason(reason):
                self.pending=(p.FAILED,reason,None)
                self.state=REPORT
        except Exception as exc:
            self.pending=(p.FAILED,"VISION_ERROR",None)
            self.last_reason=type(exc).__name__+":"+str(exc)
            if not self.job.is_scan:self._record_measurement_reason(self.last_reason)
            self.state=REPORT

    def flush_report(self):
        # Main pumps RX AGAIN before calling this, including after heavy inference.
        if self.state!=REPORT or self.pending is None: return
        status,reason,observation=self.pending
        if status in (p.SUCCESS,p.OBSERVATION_ONLY) and observation is not None:
            try: self.factory.completed(self.request,observation)
            except Exception:
                status,reason,observation=p.FAILED,"VISION_ERROR",None
        self._finish_active(status,reason,observation)

    def fail_active(self,reason):
        if self.request is None: return
        self.pending=(p.FAILED,reason,None)
        self.state=REPORT

    def reject_command(self,frame,reason):
        now=self.clock()
        key=(frame.seq,frame.cmd,bytes(frame.payload))
        if self._seen(key,now): return
        self._remember(key,now)
        request={"cmd":frame.cmd,"input_seq":frame.seq,"target_id":None,"slot_id":None,"key":key}
        self._transmit_once(request,self._new_run_id(),p.FAILED,reason)

    def close(self):
        # Program exit is silent; no unrequested packet during resource teardown.
        self._discard_active()
