"""Competition K230 entry: IDLE -> command -> observe/average -> one report -> IDLE.

Original photograph application is preserved byte-for-byte as capture_tool.py.
No STM32 code, motion, laser control or autonomous vehicle task sequence lives here.
"""
import gc
import os
import sys
import time
if "/sdcard" not in sys.path: sys.path.append("/sdcard")
import runtime_config as cfg
if cfg.PROJECT_ROOT not in sys.path: sys.path.append(cfg.PROJECT_ROOT)
from camera_service import CameraService,ticks_ms,ticks_diff
from task_manager import TaskManager,IDLE,REPORT
from task_runner import VisionTaskFactory
from calibration_params import load_calibration
import protocol as protocol
from user_button import UserButton
from run_control import RunControl
from vision_debug import status_text,target_label


class UnavailableFactory:
    camera_enabled=False
    def start(self,request): raise ValueError("CALIBRATION_NOT_READY")
    def close(self): pass


def pump_commands(link,manager):
    # STOP is returned before the normal queue and clears queued old starts.
    priority=link.pop_priority()
    if priority is not None: manager.on_command(priority)
    overflow=link.take_overflow()
    if overflow is not None:
        if manager.request is not None: manager.fail_active("COMMAND_QUEUE_OVERFLOW")
        else: manager.reject_command(overflow,"COMMAND_QUEUE_OVERFLOW")
        return
    for _ in range(cfg.COMMANDS_PER_POLL):
        frame=link.pop_command()
        if frame is None: break
        manager.on_command(frame)
    if not link.healthy(): manager.fail_active("LINK_ERROR")


def debug_preview(camera,manager,control,image=None):
    renderer=getattr(camera,"render_debug",None)
    if renderer is None:
        if image is not None:camera.show(image)
        return
    state=control.state if control is not None and not control.running else manager.state
    labels={"WAIT_START":"等待启动","FAULT":"故障","IDLE":"空闲",
            "SETTLING":"等待停稳","RUNNING":"识别中","REPORT":"准备回传","PAUSED":"已停止"}
    request=manager.request
    cmd=request["cmd"] if request else None
    count=len(manager.average.samples) if manager.average is not None else 0
    error=getattr(control,"error",None) if control is not None else None
    reason=error or manager.last_reason
    display_cmd=cmd
    if display_cmd is None and manager.last_report is not None:
        display_cmd=manager.last_report["request"]["cmd"]
    task_labels={protocol.QR_START:"二维码定位",protocol.BOMB_SCAN:"排爆顺序扫描",protocol.BOMB_MEASURE:"爆炸物球心定位",
                 protocol.AIM_SCAN:"靶标顺序扫描",protocol.AIM_MEASURE:"靶心定位",protocol.RESCUE_SCAN:"人质顺序扫描",protocol.RESCUE_MEASURE:"人质定位"}
    lines=["K230 视觉调试",labels.get(state,state),
           ("任务:" if cmd is not None else "上次任务:")+task_labels.get(display_cmd,"--"),
           "采样:%d/%d" % (count,cfg.MEASUREMENT_SAMPLES),
           "帧:%d" % getattr(camera,"frame_id",0),"原因:",status_text(reason),"代码:",reason]
    if display_cmd==protocol.AIM_SCAN:
        lines[3]="扫描已回传"
        if cmd==protocol.AIM_SCAN and manager.job is not None:
            task=manager.job.task
            if hasattr(task,"hits") and hasattr(task,"min_hits"):
                lines[3]="识别:%d/%d" % (task.hits,task.min_hits)
            else:lines[3]="靶标扫描"
            target=request.get("target_id")
            lines.insert(4,"目标:"+target_label("aim",target) if type(target) is int and 1<=target<=3 else "目标:未选择")
    if cmd==protocol.QR_START:
        lines.insert(3,"QR:%dx%d" % cfg.QR_CAPTURE_SIZE if cfg.QR_HIGH_RES_ENABLED else "QR:640x480")
        lines.insert(4,"采样窗口:%.1fs" % (cfg.QR_SAMPLE_WINDOW_MS/1000.0))
    selection=getattr(manager.factory,"selection",None)
    if selection:lines.append("任务码:"+selection.get("code","已缓存"))
    last=manager.last_report
    if last is not None:
        obs=last.get("observation") or {}
        lines.extend(["上次结果:",protocol.NAMES.get(last["request"]["cmd"],"--"),
                      "状态:%d" % last["status"],
                      "序号:%d" % (obs.get("target_slot",obs.get("slot_id")) or 0)])
        point=obs.get("position_tool_mm")
        if point is not None:
            lines.extend(["%s:%+.1f mm" % (axis,value) for axis,value in zip("XYZ",point)])
            lines.append("散布:%.2f mm" % obs.get("scatter_mm",0.0))
    order=getattr(manager,"last_scan_order",None)
    if order:lines.append("已确认顺序:"+",".join(str(value) for value in order))
    diagnostic=getattr(manager,"last_scan_error",None)
    if diagnostic and diagnostic!=reason:lines.extend(["扫描诊断:",diagnostic])
    gate=None
    if cmd==protocol.QR_START and manager.job is not None:
        size=cfg.QR_CAPTURE_SIZE if cfg.QR_HIGH_RES_ENABLED else manager.job.task.estimator.image_size
        roi=manager.job.task.detection_roi(size)
        if roi is not None:
            x,y,w,h=roi
            gate=(int(round(x*640.0/size[0])),int(round(y*480.0/size[1])),
                  max(1,int(round(w*640.0/size[0]))),max(1,int(round(h*480.0/size[1]))))
            lines.insert(5,"预览ROI:%dx%d" % (gate[2],gate[3]))
    elif cmd in (protocol.BOMB_SCAN,protocol.BOMB_MEASURE):
        from config import SCAN_GATE_RAW_PX
        gate=SCAN_GATE_RAW_PX
    elif cmd in (protocol.AIM_SCAN,protocol.AIM_MEASURE):
        from aim_config import SCAN_GATE_RAW_PX
        gate=SCAN_GATE_RAW_PX
    elif cmd==protocol.RESCUE_SCAN:
        from rescue_config import SCAN_GATE_RAW_PX
        gate=SCAN_GATE_RAW_PX
    renderer(image,{"lines":lines,"scan_gate":gate})


def run_loop(link,manager,camera,max_loops=None,exitpoint=None,sleep_ms=None,control=None):
    loops,last_preview=0,ticks_ms()
    last_qr_log=last_preview
    frame=None
    while max_loops is None or loops<max_loops:
        loops+=1
        if exitpoint is not None: exitpoint()
        if control is not None:
            control.poll()
            if not control.running:
                if ticks_diff(ticks_ms(),last_preview)>=cfg.DISPLAY_REFRESH_MS or control.error:
                    debug_preview(camera,manager,control)
                    last_preview=ticks_ms()
                if control.error or not link.healthy(): break
                if sleep_ms is not None: sleep_ms(cfg.LOOP_SLEEP_MS)
                continue
        pump_commands(link,manager)
        manager.tick()
        frame=None
        capture_generation=control.generation if control is not None else None
        capture_request_key=manager.request["key"] if manager.request is not None else None
        try:
            # Keep capture warmed even in IDLE; no inference or result packets in IDLE.
            if manager.state!=REPORT and getattr(manager.factory,"camera_enabled",True):
                select_qr=getattr(camera,"set_qr_capture",None)
                if select_qr is not None:
                    select_qr(manager.request is not None and manager.request["cmd"]==protocol.QR_START)
                frame=camera.get_frame()
                manager.observe_camera_epoch(camera.epoch)
                if camera.state=="ERROR": manager.fail_active("CAMERA_ERROR")
        except Exception:
            manager.fail_active("CAMERA_ERROR")
        if control is not None:
            control.poll()
            if not control.running or control.generation!=capture_generation:
                frame=None
                continue
        pump_commands(link,manager)
        manager.tick()
        current_key=manager.request["key"] if manager.request is not None else None
        if frame is not None and capture_request_key==current_key:
            manager.poll_frame(frame)
        # A STOP/new command received during snapshot or inference wins before report.
        if control is not None:
            control.poll()
            if not control.running or control.generation!=capture_generation:
                frame=None
                continue
        pump_commands(link,manager)
        manager.tick()
        manager.flush_report()
        if not link.healthy():
            print("K230_LINK_ERROR",link.diagnostics())
            break
        if ticks_diff(ticks_ms(),last_preview)>=cfg.DISPLAY_REFRESH_MS:
            debug_preview(camera,manager,control,frame["image"] if frame is not None else None)
            last_preview=ticks_ms()
        if (manager.request is not None and manager.request["cmd"]==protocol.QR_START and
                ticks_diff(ticks_ms(),last_qr_log)>=1000):
            print("K230_QR_STATE",manager.last_reason,"samples",
                  len(manager.average.samples),"required",cfg.MEASUREMENT_SAMPLES)
            last_qr_log=ticks_ms()
        frame=None
        gc.collect()
        if sleep_ms is not None: sleep_ms(cfg.LOOP_SLEEP_MS)


def main(max_loops=None):
    link=factory=camera=manager=button=control=None
    try:
        from vision_link import VisionLink
        link=VisionLink(uart_id=cfg.UART_ID,baudrate=cfg.UART_BAUDRATE,tx_pin=cfg.UART_TX_PIN,
                        rx_pin=cfg.UART_RX_PIN,queue_size=cfg.COMMAND_QUEUE_SIZE,
                        priority_cmds=(protocol.STOP,),read_chunk=cfg.RX_READ_BYTES)
        try:
            calibration=load_calibration(cfg.PROJECT_ROOT,handeye_accepted_override=cfg.HANDEYE_ACCEPTED_BY_USER)
            factory=VisionTaskFactory(calibration,cfg.PROJECT_ROOT)
        except (OSError,ValueError,KeyError) as exc:
            print("CALIBRATION_NOT_READY",str(exc))
            factory=UnavailableFactory()
        camera=CameraService(qr_enabled=cfg.QR_HIGH_RES_ENABLED)
        manager=TaskManager(link,factory)
        button=UserButton()
        control=RunControl(link,manager,button)
        print("K230_RUNTIME","WAIT_START","GPIO",cfg.USER_KEY_GPIO,"UART%d %d" % (cfg.UART_ID, cfg.UART_BAUDRATE),
              "mean_samples",cfg.MEASUREMENT_SAMPLES)
        print("QR_CAPTURE_SIZE",cfg.QR_CAPTURE_SIZE if cfg.QR_HIGH_RES_ENABLED else (640,480),
              "RGB_PREVIEW_YOLO_SIZE",(640,480))
        print("QR_SAMPLE_WINDOW_MS",cfg.QR_SAMPLE_WINDOW_MS,"QR_CONTRAST_RETRY",
              cfg.QR_CONTRAST_RETRY_ENABLED,"EVERY",cfg.QR_CONTRAST_RETRY_EVERY_N_FRAMES)
        debug_preview(camera,manager,control)
        run_loop(link,manager,camera,max_loops,os.exitpoint,time.sleep_ms,control)
    except KeyboardInterrupt:
        print("K230_RUNTIME_STOPPED")
    finally:
        try:
            if control is not None: control.close()
        finally:
            try:
                if manager is not None: manager.close()
            finally:
                try:
                    if button is not None and not button.close(): print("BUTTON_SHUTDOWN_TIMEOUT")
                finally:
                    try:
                        if factory is not None: factory.close()
                    finally:
                        try:
                            if camera is not None: camera.close()
                        finally:
                            if link is not None and not link.deinit(): print("UART_SHUTDOWN_TIMEOUT")


if __name__=="__main__":
    main()
