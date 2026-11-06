"""Standalone K230 target observation. No UART, laser, chassis or arm commands."""
import gc
import os
import sys
import time
PROJECT_ROOT = "/data/guanggongsai"
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)
from calibration_params import load_calibration
from tasks.task_aim import AimTask
from aim_config import SCAN_GATE_RAW_PX


def main(mode="measure", color_id=1, max_frames=180,
         scan_order_confirmed=False, start_boundary_confirmed=False):
    if mode not in ("scan", "measure"):
        raise ValueError("mode must be scan or measure")
    if mode == "scan" and not (scan_order_confirmed is True and start_boundary_confirmed is True):
        return {"ok": False, "reason": "SCAN_ORDER_NOT_CONFIRMED"}
    task = AimTask(load_calibration(PROJECT_ROOT))
    if mode == "scan":
        task.start_scan(color_id, scan_order_confirmed, start_boundary_confirmed, max_frames=max_frames)
    else:
        task.start_measure(color_id, max_frames=max_frames)
    sensor = raw = preview = None
    display_touched = media_touched = False
    frame_id, last_print = 0, time.ticks_ms()
    result = {"ok": False, "reason": "TARGET_NOT_FOUND"}
    try:
        from media.sensor import Sensor
        from media.display import Display
        from media.media import MediaManager, CAM_CHN_ID_0
        sensor = Sensor(id=0)
        sensor.reset()
        sensor.set_framesize(Sensor.VGA, chn=CAM_CHN_ID_0)
        sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_0)
        display_touched = True
        Display.init(Display.ST7701, width=800, height=480, to_ide=False)
        media_touched = True
        MediaManager.init()
        sensor.run()
        for _ in range(10):
            os.exitpoint()
            raw = sensor.snapshot(chn=CAM_CHN_ID_0, timeout=2000, auto_reboot=False)
            raw = None
        while frame_id < max_frames:
            os.exitpoint()
            raw = sensor.snapshot(chn=CAM_CHN_ID_0, timeout=2000, auto_reboot=False)
            sampled_ms = time.ticks_ms()
            frame_id += 1
            # In measure mode the camera MUST already be physically stationary.
            result = task.poll(raw, frame_id, geometry_confirmed=True, stable=(mode == "measure"))
            result["capture_return_ms"] = sampled_ms
            preview = raw.to_rgb565()
            if mode == "scan":
                preview.draw_rectangle(SCAN_GATE_RAW_PX, color=(255,255,0), thickness=2)
            elif result.get("position_camera_mm") is not None:
                u, v = task.calibration.camera.project_ray(result["position_camera_mm"])
                preview.draw_cross(int(u),int(v),color=(0,255,0),size=12,thickness=2)
            Display.show_image(preview)
            now = time.ticks_ms()
            if time.ticks_diff(now,last_print) >= 500:
                print("AIM",frame_id,result["reason"],"p_E_mm",result.get("position_tool_mm"),
                      "slot",result.get("target_slot"))
                last_print = now
            if result.get("complete") or result.get("measurement_complete"):
                print("AIM_RESULT",result)
                break
            raw = preview = None
            gc.collect()
        else:
            result = {"ok":False,"reason":"TARGET_TASK_FRAME_BUDGET_EXCEEDED",
                      "position_tool_mm":None,"control_ready":False}
        return result
    finally:
        raw = preview = None
        try:
            task.close()
        finally:
            try:
                if sensor is not None: sensor.stop()
            finally:
                try:
                    if display_touched: Display.deinit()
                finally:
                    gc.collect()
                    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
                    time.sleep_ms(100)
                    if media_touched: MediaManager.deinit()


if __name__ == "__main__":
    main()
