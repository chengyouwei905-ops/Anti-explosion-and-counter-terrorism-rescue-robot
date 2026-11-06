"""K230 rescue diagnostics and observation only. No UART or robot commands."""
import gc
import os
import sys
import time
try:
    import ujson as json
except ImportError:
    import json
PROJECT_ROOT="/data/guanggongsai"
if PROJECT_ROOT not in sys.path: sys.path.append(PROJECT_ROOT)
from calibration_params import load_calibration
from rescue_config import MODEL_TEMPLATE,LAYOUT_TEMPLATE,SCAN_GATE_RAW_PX,MAX_TABLE_CANDIDATES,RECTANGLE_THRESHOLD
from rescue_detector import RescueYoloDetector
from rescue_reference import RescueLayout,NativeRescueTableReference
from tasks.task_rescue import RescueTask
from tasks.task_rescue_object import RescueObjectTask
import runtime_config as runtime_cfg


def main(mode="inspect",shape_id=1,slot_id=None,max_frames=120,detector_backend=None,
         scan_order_confirmed=False,start_boundary_confirmed=False):
    """inspect works without a model; it shows rectangle candidates, no 3D result."""
    if mode not in ("inspect","table","scan","measure"):
        raise ValueError("mode must be inspect, table, scan or measure")
    if mode=="scan" and not (scan_order_confirmed is True and start_boundary_confirmed is True):
        return {"ok":False,"reason":"SCAN_ORDER_NOT_CONFIRMED"}
    if type(max_frames) is not int or max_frames<1: raise ValueError("INVALID_FRAME_BUDGET")
    cal=load_calibration(PROJECT_ROOT,handeye_accepted_override=runtime_cfg.HANDEYE_ACCEPTED_BY_USER)
    with open(PROJECT_ROOT+"/"+LAYOUT_TEMPLATE,"r") as stream: layout=RescueLayout(json.load(stream))
    reference=NativeRescueTableReference(cal,layout)
    detector=task=sensor=raw=preview=gray=None
    display_touched=media_touched=False
    frame_id,last_print=0,0
    result={"ok":False,"reason":"NO_OBSERVATION","position_tool_mm":None}
    try:
        if mode in ("scan","measure"):
            with open(PROJECT_ROOT+"/"+MODEL_TEMPLATE,"r") as stream: metadata=json.load(stream)
            if metadata.get("model_path") and not metadata["model_path"].startswith("/"):
                metadata["model_path"]=PROJECT_ROOT+"/"+metadata["model_path"]
            if detector_backend is None and metadata.get("model_path"):
                detector_backend=runtime_cfg.create_model_backend("rescue",metadata)
            detector=RescueYoloDetector(metadata,detector_backend)
            loaded=detector.load()
            if not loaded["ok"]: return loaded
            object_mode=runtime_cfg.RESCUE_REFERENCE_MODE=="object"
            task=RescueObjectTask(cal,detector) if object_mode else RescueTask(cal,detector,layout,reference)
            if mode=="scan": task.start_scan(shape_id,True,True,max_frames=max_frames)
            else:
                if not object_mode:
                    if slot_id is None: return {"ok":False,"reason":"TARGET_SLOT_NOT_KNOWN"}
                    if reference.readiness()!="OK": return {"ok":False,"reason":reference.readiness()}
                task.start_measure(shape_id,slot_id,max_frames=max_frames)
        elif mode=="table" and reference.readiness()!="OK":
            return {"ok":False,"reason":reference.readiness()}
        last_print=time.ticks_ms()
        from media.sensor import Sensor
        from media.display import Display
        from media.media import MediaManager,CAM_CHN_ID_0
        sensor=Sensor(id=0)
        sensor.reset()
        sensor.set_framesize(Sensor.VGA,chn=CAM_CHN_ID_0)
        sensor.set_pixformat(Sensor.RGB888,chn=CAM_CHN_ID_0)
        display_touched=True
        Display.init(Display.ST7701,width=800,height=480,to_ide=False)
        media_touched=True
        MediaManager.init()
        sensor.run()
        for _ in range(10):
            os.exitpoint()
            raw=sensor.snapshot(chn=CAM_CHN_ID_0,timeout=2000,auto_reboot=False)
            raw=None
        while frame_id<max_frames:
            os.exitpoint()
            raw=sensor.snapshot(chn=CAM_CHN_ID_0,timeout=2000,auto_reboot=False)
            sampled_ms=time.ticks_ms()
            frame_id+=1
            preview=raw.to_rgb565()
            if mode=="inspect":
                gray=raw.to_grayscale()
                rects=gray.find_rects(threshold=RECTANGLE_THRESHOLD)
                candidates=[list(r.corners()) for r in rects[:MAX_TABLE_CANDIDATES]]
                result={"ok":False,"reason":"TABLE_CANDIDATES_ONLY","frame_id":frame_id,
                        "corners_candidates_raw_px":candidates,"candidate_count":len(rects),
                        "position_tool_mm":None,"control_ready":False}
                for index,corners in enumerate(candidates):
                    for i,p in enumerate(corners):
                        next_point=corners[(i+1)%4]
                        preview.draw_line(int(p[0]),int(p[1]),int(next_point[0]),int(next_point[1]),color=(0,255,0),thickness=2)
                        preview.draw_string_advanced(int(p[0]),int(p[1]),14,str(index)+":"+str(i),color=(255,255,0))
            elif mode=="table":
                result=reference.observe(raw,frame_id,True)
                result["control_ready"]=False
            else:
                result=task.poll(raw,frame_id,geometry_confirmed=True,stable=(mode=="measure"))
                if mode=="scan": preview.draw_rectangle(SCAN_GATE_RAW_PX,color=(255,255,0),thickness=2)
            result["capture_return_ms"]=sampled_ms
            if mode in ("scan","measure"):
                lines=["RESCUE "+mode,"shape: %d"%shape_id,result["reason"]]
                if result.get("position_tool_mm") is not None:
                    lines.extend(["%s: %+.1f mm"%(axis,value) for axis,value in zip("XYZ",result["position_tool_mm"])])
                for index,text in enumerate(lines):
                    preview.draw_string_advanced(5,5+index*20,16,text,color=(255,255,0))
            Display.show_image(preview)
            now=time.ticks_ms()
            if time.ticks_diff(now,last_print)>=500:
                print("RESCUE",frame_id,result["reason"],"slot",result.get("target_slot"),
                      "p_E_mm",result.get("position_tool_mm"))
                if mode=="inspect": print("TABLE_CORNERS",result["corners_candidates_raw_px"])
                last_print=now
            if result.get("complete") or result.get("measurement_complete") or (mode=="table" and result["ok"]):
                break
            raw=preview=gray=None
            gc.collect()
        else:
            if mode!="inspect":
                result={"ok":False,"reason":"RESCUE_TASK_FRAME_BUDGET_EXCEEDED",
                        "position_tool_mm":None,"control_ready":False}
        return result
    finally:
        raw=preview=gray=None
        try:
            try:
                if task is not None:
                    closer=getattr(task,"close",None)
                    if closer is not None:closer()
                    else:task.cancel()
            finally:
                try:
                    if detector is not None:detector.close()
                finally:reference.close()
        finally:
            try:
                if sensor is not None: sensor.stop()
            finally:
                try:
                    if display_touched: Display.deinit()
                finally:
                    gc.collect()
                    # No media module was imported for preflight failures.
                    if display_touched or media_touched or sensor is not None:
                        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
                        time.sleep_ms(100)
                    if media_touched: MediaManager.deinit()


if __name__=="__main__":
    # Change shape_id to 1=cylinder, 2=frustum, 3=drum for a stationary check.
    main(mode="measure",shape_id=1)
