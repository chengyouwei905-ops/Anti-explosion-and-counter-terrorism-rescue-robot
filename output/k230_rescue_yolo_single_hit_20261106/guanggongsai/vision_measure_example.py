"""Standalone board observation example. Does not replace main.py or use UART."""
import gc
import os
import sys
import time

PROJECT_ROOT = "/data/guanggongsai"
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from calibration_params import load_calibration
from tasks.task_qr import QRTask
from tasks.task_bomb import BombTask
from yolo_detector import YoloDetector
try:
    import ujson as json
except ImportError:
    import json


def main(mode="qr", max_frames=120, target_color_id=1, detector_backend=None):
    """Modes qr and bomb_refine. Camera is physically stationary for this demo.

    Future MCU integration invokes task APIs directly. This example has no
    automatic scan pass and never moves the robot or controls the laser.
    """
    if mode not in ("qr", "bomb_refine"):
        raise ValueError("mode must be qr or bomb_refine")
    calibration = load_calibration(PROJECT_ROOT)
    print("HANDEYE_ACCEPTED", calibration.handeye_accepted)
    if mode == "qr":
        task = QRTask(calibration)
    else:
        with open(PROJECT_ROOT+"/models/bomb_three_colors.template.json", "r") as stream:
            metadata = json.load(stream)
        if metadata.get("model_path") and not metadata["model_path"].startswith("/"):
            metadata["model_path"] = PROJECT_ROOT+"/"+metadata["model_path"]
        detector = YoloDetector(metadata, detector_backend)
        loaded = detector.load()
        if not loaded["ok"]:
            print("BOMB_NOT_READY", loaded["reason"])
            return loaded
        task = BombTask(calibration, detector)
        task.start_refine(target_color_id)

    sensor = raw = gray = preview = None
    display_touched = media_touched = False
    frame_id, last_print = 0, time.ticks_ms()
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
            raw = sensor.snapshot(chn=CAM_CHN_ID_0, timeout=2000, auto_reboot=False)
            raw = None
        while max_frames is None or frame_id < max_frames:
            os.exitpoint()
            raw = sensor.snapshot(chn=CAM_CHN_ID_0, timeout=2000, auto_reboot=False)
            frame_id += 1
            sampled_ms = time.ticks_ms()  # Return time, not exposure time.
            if mode == "qr":
                gray = raw.to_grayscale()
                result = task.measure(gray, frame_id, geometry_confirmed=True, stable=True)
                preview=gray.to_rgb565()
                Display.show_image(preview)
            else:
                result = task.poll(raw, frame_id, geometry_confirmed=True, stable=True)
                preview=raw.to_rgb565()
                Display.show_image(preview)
            result["capture_return_ms"] = sampled_ms
            now = time.ticks_ms()
            if time.ticks_diff(now, last_print) >= 500:
                # Large matrices are intentionally not printed every frame.
                print("VISION", frame_id, result["reason"], "p_E_mm", result.get("position_tool_mm"),
                      "control_ready", result["control_ready"])
                last_print = now
            raw = gray = preview = None
            gc.collect()
    finally:
        raw = gray = preview = None
        try:
            if isinstance(task, BombTask):
                task.close()
        finally:
            try:
                if sensor is not None:
                    sensor.stop()
            finally:
                try:
                    if display_touched:
                        Display.deinit()
                finally:
                    gc.collect()
                    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
                    time.sleep_ms(100)
                    if media_touched:
                        MediaManager.deinit()


if __name__ == "__main__":
    main()
