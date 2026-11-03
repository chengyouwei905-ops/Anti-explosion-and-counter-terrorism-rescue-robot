"""Four-target stationary field test, compatible with the existing MCU QR callback.

All test packets deliberately use task_cmd=0x10, slot_id=0 as a DEBUG channel.
XYZ is the actual selected target reference in calibrated E, never replaced
by the expected value. This is not the formal competition command semantics.
The MCU's qr_position_body then means CURRENT TARGET position in B.
Run only one field_*_tcp_test.py at a time, separate from competition_main.
"""
import gc
import os
import sys
if "/data/guanggongsai" not in sys.path:sys.path.insert(0,"/data/guanggongsai")
import runtime_config as cfg
import protocol as p
from camera_service import CameraService,ticks_ms,ticks_diff
from calibration_params import load_calibration
from measurement import MeasurementAverage,QRMeasurementAverage
from task_runner import VisionTaskFactory
from qr_tcp_stream import QRStream
from vision_link import VisionLink,_sleep_ms
from field_tcp_reference import PROFILES,print_reference,tcp_to_body
from vision_debug import target_label,status_text
from boundary_policy import enabled as boundary_checks


class FieldTCPStream(QRStream):
    def __init__(self,job,link,kind,target_id=1,sample_count=3,send_interval_ms=1000,
                 sample_window_ms=3000,arm_pose=None,body_reference=None,
                 clock=ticks_ms,diff=ticks_diff):
        if kind not in PROFILES:raise ValueError("INVALID_FIELD_TEST_KIND")
        if type(target_id) is not int or not 1<=target_id<=3:raise ValueError("INVALID_TARGET_ID")
        if type(send_interval_ms) is not int or send_interval_ms<1000:
            raise ValueError("FIELD_SEND_INTERVAL_MUST_BE_AT_LEAST_1000_MS")
        super().__init__(job,link,clock,diff)
        self.kind,self.target_id=kind,target_id
        self.required_samples,self.send_interval_ms=sample_count,send_interval_ms
        self.arm_pose=arm_pose
        self.body_reference=PROFILES[kind]["body_reference"] if body_reference is None else body_reference
        self.average=(QRMeasurementAverage(sample_count,8.,sample_window_ms,clock,diff)
                      if kind=="qr" and sample_window_ms>0 else MeasurementAverage(sample_count,8.))
        self.debug_view=None
        self.debug_pose_valid=None
        enable=getattr(job,"enable_debug",None)
        if enable is not None:enable()

    def _clear_debug(self):
        self.debug_view=None
        self.debug_pose_valid=None

    def _sync_debug(self):
        snapshot=getattr(self.task,"debug_snapshot",None)
        self.debug_view=snapshot() if snapshot is not None else None

    def _preview_only(self,frame):
        preview=getattr(self.task,"preview",None)
        if preview is not None:preview(frame)
        self._sync_debug()

    def _reset_job(self):
        self.task.reset_window()

    def resume(self,input_seq=None):
        self._reset_job()
        self._clear_debug()
        self.last_observation=None
        super().resume(input_seq)

    def pause(self):
        self._reset_job()
        self._clear_debug()
        super().pause()

    def process_frame(self,frame):
        self._clear_debug()
        if frame["epoch"]!=self.epoch:
            self.epoch=frame["epoch"]
            self.average.reset()
            self._reset_job()
            self.started_ms=self.clock()
        if not self.active:return None
        now=self.clock()
        if self.diff(now,self.started_ms)<250:
            self._preview_only(frame)
            self.last_reason="SETTLING";return None
        if self.last_send_ms is not None and self.diff(now,self.last_send_ms)<self.send_interval_ms:
            self._preview_only(frame)
            self.last_reason="WAIT_NEXT_BATCH";return None
        observed=self.task.poll(frame)
        self._sync_debug()
        self.debug_pose_valid=observed.get("ok") is True
        observed.setdefault("reference_point",PROFILES[self.kind]["reference_point"])
        if isinstance(self.average,QRMeasurementAverage):
            averaged,self.last_reason=self.average.add(observed,frame.get("capture_return_ms"))
        else:
            averaged,self.last_reason=self.average.add(observed)
        if not observed.get("ok") and self.kind!="qr":self._reset_job()
        if averaged is not None:
            self.average.reset()
            self._reset_job()
        return averaged

    def send_observation(self,observation):
        if not self.active:return False
        # Wire-only alias: preserve real task metadata in logs and samples.
        wire=dict(observation)
        wire["slot_id"]=0
        wire.pop("target_slot",None)
        wire.pop("normal_tool",None)
        wire.pop("grasp_position_tool_mm",None)
        wire.pop("quaternion_tool_from_qr",None)
        ready=wire.get("control_ready") is True
        status=p.SUCCESS if ready else p.OBSERVATION_ONLY
        reason="OK" if ready else wire.get("reason","HANDEYE_CANDIDATE_ONLY")
        request={"cmd":p.QR_START,"input_seq":self.input_seq,"slot_id":0}
        payload=p.encode_result(request,self.run_id,status,reason,wire)
        self.link.send(p.RESULT,payload)
        self.last_send_ms=self.clock()
        self.last_observation=observation
        self.sent_count+=1
        self.last_reason="SENT" if ready else "OBSERVATION_ONLY"
        point=observation["position_tool_mm"]
        print("FIELD_TCP_TX",self.kind,"target_id",self.target_id,"packet",self.sent_count,
              "source_cmd",PROFILES[self.kind]["source_cmd"],"wire_cmd",p.QR_START,
              "p_E_mm",tuple(point),"samples",observation["sample_count"],
              "scatter_mm",observation["scatter_mm"])
        if self.arm_pose is not None:
            body=tcp_to_body(point,self.arm_pose)
            print("BODY_USING_CONFIGURED_POSE_MM",tuple(body))
            if self.body_reference is not None:
                print("BODY_MINUS_CONFIGURED_REFERENCE_MM",
                      tuple(body[i]-self.body_reference[i] for i in range(3)))
        return True


def pump_controls(link,stream,button=None):
    if not link.healthy():raise OSError("FIELD_LINK_ERROR:"+str(link.diagnostics()))
    if button is not None:
        if not button.healthy():raise OSError("FIELD_BUTTON_ERROR:"+str(button.error))
        while button.pressed():
            if stream.active:stream.pause()
            else:stream.resume()
            print("FIELD_STREAM",stream.kind,"RUNNING" if stream.active else "PAUSED")
    stop=link.pop_priority()
    if stop is not None:
        if not stop.payload:stream.pause()
        else:print("FIELD_BAD_STOP_PAYLOAD")
        return
    if link.take_overflow() is not None:raise OSError("FIELD_COMMAND_QUEUE_OVERFLOW")
    for _ in range(cfg.COMMANDS_PER_POLL):
        command=link.pop_command()
        if command is None:break
        if command.cmd==p.QR_START and not command.payload:
            stream.resume(command.seq)
            print("FIELD_MCU_REQUEST",command.seq,"reset samples",stream.kind)
        else:print("FIELD_IGNORED_CMD",command.cmd)


def render_preview(camera,stream,image=None,frame_id=None):
    view=getattr(stream,"debug_view",None)
    current=view is not None and frame_id is not None and view["frame_id"]==frame_id
    detections=view["detections"] if current else []
    selected=sum(item.get("selected",False) for item in detections)
    pose_valid=getattr(stream,"debug_pose_valid",None) if current else None
    lines=[PROFILES[stream.kind]["label"]+"测试","目标:"+target_label(stream.kind,stream.target_id),
           "运行中" if stream.active else "已暂停",
           "边界限制:"+("开启" if boundary_checks() else "关闭"),
           "检测:%d 目标:%d" % (len(detections),selected),
           "定位:"+("有效" if pose_valid is True else ("未通过" if pose_valid is False else "待采样")),
           status_text(stream.last_reason),
           "采样:%d/%d" % (len(stream.average.samples),stream.required_samples),
           "已发送:%d" % stream.sent_count]
    if current and view.get("error"):lines.append("检测反馈异常")
    if stream.last_observation is not None:
        age=stream.diff(stream.clock(),stream.last_send_ms)
        lines.extend(["已发TCP(mm)","%dms前" % age])
        lines.extend("%s:%+.1fmm" % (axis,value) for axis,value in
                     zip("XYZ",stream.last_observation["position_tool_mm"]))
    status={"lines":lines,"detections":detections,
            "source_size":view["source_size"] if current else (640,480),
            "pose_valid":pose_valid,"clear_preview":image is None}
    if stream.kind=="bomb":
        from config import BOMB_DETECTION_ROI_RAW_PX
        status["scan_gate"]=BOMB_DETECTION_ROI_RAW_PX
    elif stream.kind=="aim":
        from aim_config import TARGET_DETECTION_ROI_RAW_PX
        status["scan_gate"]=TARGET_DETECTION_ROI_RAW_PX
    camera.render_debug(image,status)


def run_loop(link,stream,camera,button=None,max_loops=None,exitpoint=None,sleep_ms=_sleep_ms):
    loops=0
    last_preview=last_log=stream.clock()
    while max_loops is None or loops<max_loops:
        loops+=1
        if exitpoint is not None:exitpoint()
        pump_controls(link,stream,button)
        camera.set_qr_capture(stream.active and stream.kind=="qr")
        generation=stream.generation
        frame=camera.get_frame()
        if camera.state=="ERROR":raise OSError(camera.error or "CAMERA_ERROR")
        pump_controls(link,stream,button)
        if frame is None:
            stream.average.reset()
            stream._reset_job()
            stream._clear_debug()
            stream.last_reason="CAMERA_"+camera.state if stream.active else "PAUSED"
        elif generation==stream.generation:
            observed=stream.process_frame(frame)
            pump_controls(link,stream,button)
            if observed is not None and generation==stream.generation and stream.active:
                stream.send_observation(observed)
        now=stream.clock()
        if stream.diff(now,last_preview)>=200:
            render_preview(camera,stream,frame["image"] if frame is not None else None,
                           frame["frame_id"] if frame is not None else None)
            last_preview=now
        if stream.diff(now,last_log)>=1000:
            view=stream.debug_view or {}
            detections=view.get("detections",[])
            print("FIELD_TCP_STATE",stream.kind,stream.last_reason,
                  "detected",len(detections),"target",sum(d.get("selected",False) for d in detections),
                  "samples",len(stream.average.samples),"sent",stream.sent_count)
            last_log=now
        frame=None
        gc.collect()
        sleep_ms(cfg.LOOP_SLEEP_MS)


def main(kind="qr",target_id=1,arm_pose=None,body_reference=None,
         sample_count=3,send_interval_ms=1000,sample_window_ms=3000,
         auto_start=True,enable_user_button=True,max_loops=None,show_to_ide=True):
    if kind not in PROFILES:raise ValueError("INVALID_FIELD_TEST_KIND")
    if type(target_id) is not int or not 1<=target_id<=3:raise ValueError("INVALID_TARGET_ID")
    link=camera=factory=job=button=stream=None
    try:
        calibration=load_calibration(cfg.PROJECT_ROOT,handeye_accepted_override=cfg.HANDEYE_ACCEPTED_BY_USER)
        if calibration.T_tool_from_camera is None:raise ValueError("HANDEYE_NOT_CONFIGURED")
        print("FIELD_TCP_DEBUG_ALIAS: all targets use wire task_cmd=0x10, slot_id=0")
        print("No TOTALSTART; fixed chassis; pause, move arm, settle, resume.")
        if arm_pose is None:
            print("NO_ACTUAL_ARM_POSE_CONFIGURED: following TCP reference is a preset EXAMPLE only")
            print("Set ARM_POSE_FOR_REFERENCE to the actual MCU vision_arm_pose for an applicable comparison")
        print_reference(kind,arm_pose,body_reference,calibration)
        factory=VisionTaskFactory(calibration,cfg.PROJECT_ROOT)
        request={"cmd":PROFILES[kind]["source_cmd"],"input_seq":0,
                 "target_id":None if kind=="qr" else target_id,
                 "slot_id":None if kind=="qr" else 1}
        job=factory.start(request)
        link=VisionLink(uart_id=cfg.UART_ID,baudrate=cfg.UART_BAUDRATE,
                        tx_pin=cfg.UART_TX_PIN,rx_pin=cfg.UART_RX_PIN,
                        queue_size=cfg.COMMAND_QUEUE_SIZE,priority_cmds=(p.STOP,),read_chunk=cfg.RX_READ_BYTES)
        camera=CameraService(qr_enabled=kind=="qr" and cfg.QR_HIGH_RES_ENABLED,display_to_ide=show_to_ide)
        if enable_user_button:
            from user_button import UserButton
            button=UserButton()
        stream=FieldTCPStream(job,link,kind,target_id,sample_count,send_interval_ms,
                              sample_window_ms,arm_pose,body_reference)
        if auto_start:stream.resume()
        print("FIELD_TCP_READY",kind,"target_id",target_id,"samples",sample_count,
              "min_interval_ms",send_interval_ms)
        run_loop(link,stream,camera,button,max_loops,getattr(os,"exitpoint",None))
    except KeyboardInterrupt:
        print("FIELD_TCP_STOPPED",kind)
    finally:
        try:
            if stream is not None:stream.pause()
            if job is not None:job.cancel()
        finally:
            try:
                if button is not None and not button.close():print("BUTTON_SHUTDOWN_TIMEOUT")
            finally:
                try:
                    if factory is not None:factory.close()
                finally:
                    try:
                        if camera is not None:camera.close()
                    finally:
                        if link is not None and not link.deinit():print("UART_SHUTDOWN_TIMEOUT")


if __name__=="__main__":main()
