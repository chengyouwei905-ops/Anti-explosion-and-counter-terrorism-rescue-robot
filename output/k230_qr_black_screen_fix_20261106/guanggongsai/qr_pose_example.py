"""K230 / GC2093 最小相机示例；在 CanMV 中运行，导入本文件不会启相机。"""
import gc
import os
import sys
import time
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager, CAM_CHN_ID_0
# VS Code 的“运行当前文件”通过 <stdin> 执行，不保证当前目录在 SD 卡。
if "/data/guanggongsai" not in sys.path:
    sys.path.insert(0,"/data/guanggongsai")
from qr_pose import QRPoseEstimator, CAMERA_MATRIX, DIST_COEFFS, IMAGE_SIZE

# 必须填写实测编码正方形边长（mm），不能包含周围白色静区。
QR_SIZE_MM = 80.0  # 用户确认的实测编码边长，不含白色静区。
TARGET_PAYLOAD = None  # 例如 "target-01"；None 表示逐个返回全部普通 QR。


def main(max_frames=None):
    # 未填有效尺寸会在分配任何相机/显示资源前明确报错。
    estimator = QRPoseEstimator(QR_SIZE_MM, CAMERA_MATRIX, DIST_COEFFS,
                                IMAGE_SIZE, detector_order="canmv_quirc")
    sensor = raw = gray = preview = None
    display_touched = media_touched = False
    frames = detections = valid_poses = 0
    last_status = time.ticks_ms()
    try:
        # 沿用本工程标定采集的初始化：id=0、默认传感器模式、VGA、RGB888。
        # 不额外设置 hmirror/vflip/crop/传感器输入尺寸，保持标定时的默认值。
        # 若另有标定采集配置，必须把这里改成同一配置，尺寸相同并不足够。
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
            raw = sensor.snapshot(chn=CAM_CHN_ID_0)
        print("QR_POSE_READY", "640x480 raw", "qr_size_mm", QR_SIZE_MM)
        while True:
            os.exitpoint()
            raw = sensor.snapshot(chn=CAM_CHN_ID_0)
            # 仅转换颜色；不去畸变、不缩放、不裁剪、不镜像。
            gray = raw.to_grayscale()
            results = estimator.detect(gray, target_payload=TARGET_PAYLOAD,
                                       geometry_confirmed=True)
            detections += len(results)
            for result in results:
                if result["ok"]:
                    valid_poses += 1
                    print("QR", result["payload"], "t_C_from_Q_mm", result["t_camera_from_qr_mm"])
                    print("depth_mm", result["depth_mm"], "distance_mm", result["distance_mm"],
                          "reprojection_RMS_px", result["reprojection_error_px"])
                else:
                    print("QR invalid", result)
                # 用四个角画边及索引，便于检查物理方向；不使用 rect()。
                corners = result.get("corners_px")
                if corners is not None and len(corners) == 4:
                    for i in range(4):
                        x, y = int(corners[i][0]), int(corners[i][1])
                        nx, ny = int(corners[(i+1)%4][0]), int(corners[(i+1)%4][1])
                        gray.draw_line(x, y, nx, ny, color=255, thickness=2)
                        gray.draw_string_advanced(x, y, 16, str(i), color=255)
            preview=gray.to_rgb565()
            Display.show_image(preview)
            # 颜色转换产生临时图，及时回收，避免堆增长。
            raw = gray = preview = None
            gc.collect()
            frames += 1
            now = time.ticks_ms()
            if time.ticks_diff(now, last_status) >= 2000:
                print("QR_POSE_ALIVE", "frames", frames, "detections", detections,
                      "valid_poses", valid_poses, "heap_free", gc.mem_free())
                last_status = now
            if max_frames is not None and frames >= max_frames:
                print("QR_POSE_RUN_COMPLETE", frames, detections, valid_poses)
                break
    except KeyboardInterrupt:
        print("用户停止")
    finally:
        # 嵌套 finally 保证某个释放失败时仍尝试其余资源释放。
        raw = gray = preview = None
        try:
            if sensor is not None:
                sensor.stop()
        finally:
            try:
                if display_touched:
                    Display.deinit()
            finally:
                os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
                time.sleep_ms(100)
                gc.collect()
                if media_touched:
                    MediaManager.deinit()
                print("QR_POSE_RESOURCES_RELEASED")


if __name__ == "__main__":
    main()
