"""K230 dataset recorder: press GPIO53 to start, press again to finish an AVI.

Run independently of main.py. Sensor0 / VGA / RGB888, uncorrected full frames.
The LCD overlay is never encoded. No YOLO model or MCU connection is needed.
"""
import gc
import os
import sys
import time
try:
    import ustruct as struct
    import ujson as json
except ImportError:
    import struct
    import json

# Change only the group before collecting a different dataset.
DATASET_NAME = "bomb"                 # "bomb", "rescue", "target", ...
# /sdcard is the ~500 MiB firmware/examples partition. /data is the large
# writable data partition on the same card (28.7 GiB available on this board).
VIDEO_ROOT = "/data/yolo_videos"
WIDTH, HEIGHT = 640, 480
KEY_GPIO, KEY_ACTIVE = 53, 1
DEBOUNCE_MS =   40
TARGET_FPS = 10                       # Upper limit, actual FPS is measured.
JPEG_QUALITY = 90
MAX_RECORD_SECONDS = 120              # Auto-stop; press again for another clip.
MAX_CLIP_BYTES = 256 * 1024 * 1024
MIN_FREE_MB, STOP_FREE_MB = 32, 16
PREVIEW_INTERVAL_MS = 200
SYNC_INTERVAL_MS = 1000
SNAPSHOT_TIMEOUT_MS = 200
SNAPSHOT_ATTEMPTS, MAX_CAMERA_RESTARTS = 3, 2
WARMUP_FRAMES = 20
APP_VERSION = "yolo-video-capture-v1.1-data"


def ticks_ms():
    fn = getattr(time, "ticks_ms", None)
    return fn() if fn else int(time.monotonic() * 1000)


def ticks_diff(a, b):
    fn = getattr(time, "ticks_diff", None)
    return fn(a, b) if fn else a - b


def sleep_ms(value):
    fn = getattr(time, "sleep_ms", None)
    if fn:
        fn(value)
    else:
        time.sleep(value / 1000.0)


def exitpoint():
    fn = getattr(os, "exitpoint", None)
    if fn:
        fn()


def sync_storage():
    fn = getattr(os, "sync", None)
    if fn:
        fn()


def exists(path):
    try:
        os.stat(path)
        return True
    except OSError as exc:
        if exc.args and exc.args[0] == 2:
            return False
        raise


def ensure_directory(path):
    if not exists(path):
        os.mkdir(path)
    if os.stat(path)[0] & 0xF000 != 0x4000:
        raise OSError("NOT_A_DIRECTORY: " + path)


def free_megabytes(path):
    fn = getattr(os, "statvfs", None)
    if not fn:
        return None
    info = fn(path)
    return info[1] * info[4] // (1024 * 1024)


def write_all(stream, data):
    view, offset = memoryview(data), 0
    while offset < len(view):
        written = stream.write(view[offset:])
        if not written or written < 0:
            raise OSError("SHORT_FILE_WRITE")
        offset += written


class CaptureButton:
    def __init__(self, pin, clock=ticks_ms):
        self.pin, self.clock = pin, clock
        self.reset()

    def reset(self):
        self.raw = self.stable = self.pin.value()
        self.changed_at = self.clock()
        # A held key at startup/recovery must first be released.
        self.armed = self.stable != KEY_ACTIVE

    def pressed(self):
        now, value = self.clock(), self.pin.value()
        if value != self.raw:
            self.raw, self.changed_at = value, now
        if self.raw != self.stable and ticks_diff(now, self.changed_at) >= DEBOUNCE_MS:
            self.stable = self.raw
            if self.stable != KEY_ACTIVE:
                self.armed = True
            elif self.armed:
                self.armed = False
                return True
        return False


def avi_header(width, height, fps, count, movie_bytes, max_jpeg, finalized):
    """Standard single-stream MJPEG AVI; fixed 224-byte header, no audio."""
    rate = max(1, int(fps * 1000 + 0.5))
    interval = max(1, int(1000000000 / rate + 0.5))
    avih = struct.pack("<14I", interval, int(max_jpeg * fps), 0,
                       0x10 if finalized else 0, count, 0, 1, max_jpeg,
                       width, height, 0, 0, 0, 0)
    strh = struct.pack("<4s4sIHHIIIIIIIIhhhh", b"vids", b"MJPG", 0, 0, 0,
                       0, 1000, rate, 0, count, max_jpeg, 0xFFFFFFFF, 0,
                       0, 0, width, height)
    strf = struct.pack("<IiiHH4sIiiII", 40, width, height, 1, 24,
                       b"MJPG", width * height * 3, 0, 0, 0, 0)
    stream = b"strl" + b"strh" + struct.pack("<I", len(strh)) + strh
    stream += b"strf" + struct.pack("<I", len(strf)) + strf
    hdrl = b"hdrl" + b"avih" + struct.pack("<I", len(avih)) + avih
    hdrl += b"LIST" + struct.pack("<I", len(stream)) + stream
    end_size = 224 + movie_bytes + (8 + count * 16 if finalized else 0)
    header = b"RIFF" + struct.pack("<I", end_size - 8) + b"AVI "
    header += b"LIST" + struct.pack("<I", len(hdrl)) + hdrl
    header += b"LIST" + struct.pack("<I", 4 + movie_bytes) + b"movi"
    if len(header) != 224:
        raise ValueError("INVALID_AVI_HEADER")
    return header


class AviRecorder:
    """Append JPEGs and bounded index entries; publish .avi only after close."""
    def __init__(self, base, width=WIDTH, height=HEIGHT, fps=TARGET_FPS):
        self.base, self.width, self.height = base, width, height
        self.nominal_fps = fps
        self.count = self.movie_bytes = self.max_jpeg = 0
        self.index = bytearray()
        self.first_ms = self.last_ms = None
        self.file = self.csv = None
        self.closed = False
        self.path, self.pending = base + ".avi", base + ".avi.part"
        for extension in (".avi", ".avi.part", ".frames.csv", ".json"):
            if exists(base + extension):
                raise OSError("CLIP_ALREADY_EXISTS")
        try:
            self.file = open(self.pending, "w+b")
            write_all(self.file, self._header(False))
            self.csv = open(base + ".frames.csv", "w")
            self.csv.write("frame_index,capture_elapsed_ms,jpeg_bytes\n")
        except BaseException:
            if self.csv:
                self.csv.close()
            if self.file:
                self.file.close()
            self.closed = True
            raise

    def fps(self):
        span = ticks_diff(self.last_ms, self.first_ms) if self.count > 1 else 0
        return (self.count - 1) * 1000.0 / span if span > 0 else self.nominal_fps

    def _header(self, final):
        return avi_header(self.width, self.height, self.fps(), self.count,
                          self.movie_bytes, self.max_jpeg, final)

    def append(self, jpeg, captured_ms):
        if self.closed:
            raise OSError("RECORDER_CLOSED")
        size = len(jpeg)
        if size < 4 or jpeg[:2] != b"\xff\xd8" or jpeg[-2:] != b"\xff\xd9":
            raise ValueError("INVALID_JPEG")
        chunk_bytes = 8 + size + (size & 1)
        projected = 224 + self.movie_bytes + chunk_bytes + 8 + (self.count + 1) * 16
        if projected > MAX_CLIP_BYTES:
            return False
        offset = self.movie_bytes + 4  # Relative to the 'movi' list type.
        write_all(self.file, b"00dc" + struct.pack("<I", size))
        write_all(self.file, jpeg)
        if size & 1:
            write_all(self.file, b"\x00")
        self.index.extend(struct.pack("<4sIII", b"00dc", 0x10, offset, size))
        self.movie_bytes += chunk_bytes
        self.max_jpeg = max(self.max_jpeg, size)
        if self.first_ms is None:
            self.first_ms = captured_ms
        self.last_ms = captured_ms
        self.count += 1
        self.csv.write("%d,%d,%d\n" % (self.count - 1, ticks_diff(captured_ms, self.first_ms), size))
        return True

    def checkpoint(self):
        position = 224 + self.movie_bytes
        self.file.seek(0)
        write_all(self.file, self._header(False))
        self.file.seek(position)
        self.file.flush()
        self.csv.flush()
        sync_storage()

    def close(self, reason="button", encoder_name="unknown"):
        if self.closed:
            return None
        self.closed = True
        try:
            # Discard an incomplete last chunk by replacing it at the committed end.
            self.file.seek(224 + self.movie_bytes)
            write_all(self.file, b"idx1" + struct.pack("<I", len(self.index)))
            write_all(self.file, self.index)
            self.file.seek(0)
            write_all(self.file, self._header(True))
            self.file.flush()
            self.csv.flush()
        finally:
            try:
                self.file.close()
            finally:
                self.csv.close()
        sync_storage()
        if self.count:
            os.rename(self.pending, self.path)
        summary = {"version": APP_VERSION, "video": self.path if self.count else None,
                   "pending_video": None if self.count else self.pending,
                   "image_size": [self.width, self.height], "sensor_id": 0,
                   "sensor_pixel_format": "RGB888", "undistorted": False,
                   "cropped": False, "mirrored_by_script": False, "overlay_in_video": False,
                   "codec": "MJPG", "encoder": encoder_name, "jpeg_quality": JPEG_QUALITY,
                   "frame_count": self.count, "requested_fps": self.nominal_fps,
                   "measured_fps": self.fps(), "stop_reason": reason,
                   "timestamps": self.base + ".frames.csv"}
        with open(self.base + ".json", "w") as stream:
            json.dump(summary, stream)
        sync_storage()
        self.index = bytearray()
        return summary


class JpegEncoder:
    def __init__(self):
        # Verified on the actual v1.8-46 board. Its RGB888 hardware-MJPEG path
        # returns a VENC stream error, while image.to_jpeg works (~64 ms/frame).
        self.name = "image.to_jpeg"

    def encode(self, raw):
        compressed = raw.to_jpeg(quality=JPEG_QUALITY)
        return bytes(compressed.bytearray())

    def close(self):
        pass


class CaptureApp:
    def __init__(self, display=True, output_dir=None):
        self.display_enabled = display
        self.directory = output_dir
        self.sensor = self.encoder = self.recorder = self.screen = self.button = None
        self.display_touched = self.media_touched = False
        self.status, self.detail, self.error = "等待按键", "按一下开始，再按停止", False
        self.last_name = ""
        self.failures = self.restarts = 0
        self.sequence = 1
        self.record_started = self.last_sync = self.last_progress = 0

    def _new_sensor(self):
        self.sensor = self.Sensor(id=0)
        self.sensor.reset()
        self.sensor.set_framesize(self.Sensor.VGA, chn=self.channel)
        self.sensor.set_pixformat(self.Sensor.RGB888, chn=self.channel)

    def open(self):
        from machine import FPIOA, Pin
        from media.sensor import Sensor
        from media.display import Display
        from media.media import MediaManager, CAM_CHN_ID_0
        import image
        self.Sensor, self.Display, self.Media, self.channel = Sensor, Display, MediaManager, CAM_CHN_ID_0
        if not 1 <= TARGET_FPS <= 30 or not 1 <= JPEG_QUALITY <= 99:
            raise ValueError("INVALID_FPS_OR_QUALITY")
        if not DATASET_NAME or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in DATASET_NAME):
            raise ValueError("INVALID_DATASET_NAME")
        if self.directory is None:
            ensure_directory(VIDEO_ROOT)
            self.directory = VIDEO_ROOT + "/" + DATASET_NAME
        ensure_directory(self.directory)
        for name in os.listdir(self.directory):
            stem = name.split(".")[0]
            if stem.startswith("clip_") and stem[5:].isdigit():
                self.sequence = max(self.sequence, int(stem[5:]) + 1)
        fpioa = FPIOA()
        fpioa.set_function(KEY_GPIO, FPIOA.GPIO0 + KEY_GPIO)
        self.button = CaptureButton(Pin(KEY_GPIO, Pin.IN, Pin.PULL_DOWN if KEY_ACTIVE else Pin.PULL_UP))
        self._new_sensor()
        if self.display_enabled:
            self.display_touched = True
            Display.init(Display.ST7701, to_ide=True)
            if Display.width() < WIDTH + 150 or Display.height() < HEIGHT:
                raise ValueError("LCD_REQUIRES_800x480")
            self.screen = image.Image(Display.width(), Display.height(), image.RGB565)
        self.media_touched = True
        MediaManager.init()
        self.sensor.run()
        self.encoder = JpegEncoder()
        for _ in range(WARMUP_FRAMES):
            exitpoint()
            raw = self.sensor.snapshot(chn=self.channel, timeout=SNAPSHOT_TIMEOUT_MS, auto_reboot=False)
            raw = None
        self.button.reset()
        print("YOLO_CAPTURE_READY", APP_VERSION, self.directory, self.encoder.name)
        print("GPIO%d: press=start, press=stop; %dx%d, max %d FPS" % (KEY_GPIO, WIDTH, HEIGHT, TARGET_FPS))

    def start_recording(self):
        if self.recorder is not None:
            return
        free = free_megabytes(self.directory)
        if free is not None and free < MIN_FREE_MB:
            raise OSError("SD_FREE_SPACE_BELOW_%d_MB" % MIN_FREE_MB)
        base = self.directory + "/clip_%06d" % self.sequence
        self.sequence += 1
        self.recorder = AviRecorder(base)
        self.record_started = self.last_sync = self.last_progress = ticks_ms()
        self.last_name = base.split("/")[-1] + ".avi"
        self.status, self.detail, self.error = "正在录像", "再按一次保存", False
        print("REC_START", base + ".avi")

    def finish_recording(self, reason="button"):
        recorder, self.recorder = self.recorder, None
        if recorder is None:
            return
        self.status, self.detail = "正在保存", "请勿断电或拔卡"
        try:
            result = recorder.close(reason, self.encoder.name)
        except Exception as exc:
            self.status, self.detail, self.error = "保存失败", str(exc), True
            print("REC_SAVE_ERROR", recorder.pending, type(exc).__name__, str(exc))
        else:
            if result["frame_count"]:
                self.status, self.error = "已保存", False
                self.detail = "%d 帧，%.1f FPS" % (result["frame_count"], result["measured_fps"])
                print("REC_SAVED", result["video"], result["frame_count"], "frames", result["measured_fps"], "fps", reason)
            else:
                self.status, self.detail, self.error = "未录到画面", "请查看控制台", True
                print("REC_EMPTY", recorder.pending, reason)
        gc.collect()

    def read_frame(self):
        try:
            raw = self.sensor.snapshot(chn=self.channel, timeout=SNAPSHOT_TIMEOUT_MS, auto_reboot=False)
            self.failures = 0
            return raw
        except (RuntimeError, OSError) as exc:
            self.finish_recording("camera_error")
            self.status, self.detail, self.error = "相机重试", "恢复后请重新按键", True
            self.button.reset()
            self.failures += 1
            print("CAPTURE_CAMERA_ERROR", self.failures, str(exc))
            if self.failures >= SNAPSHOT_ATTEMPTS:
                if self.restarts >= MAX_CAMERA_RESTARTS:
                    raise RuntimeError("CAMERA_RECOVERY_LIMIT")
                self.restarts += 1
                old, self.sensor = self.sensor, None
                old.stop()
                old = None
                gc.collect()
                sleep_ms(200)
                self._new_sensor()
                self.sensor.run()
                sleep_ms(500)
                self.failures = 0
                self.button.reset()
            return None

    def show(self, raw):
        if self.screen is None:
            return
        # RGB565 is used only for display, after encoding the untouched RGB888.
        preview = raw.to_rgb565()
        self.screen.clear()
        self.screen.draw_image(preview, 0, 0)
        x = WIDTH + 6
        color = (255, 70, 70) if self.recorder is not None or self.error else (70, 255, 100)
        lines = [(18, 18, "YOLO 数据采集"), (60, 20, self.status),
                 (100, 16, DATASET_NAME), (132, 14, self.last_name),
                 (300, 16, "按键开始/停止"), (335, 14, "原图 640x480"),
                 (370, 14, "最长 %d 秒" % MAX_RECORD_SECONDS)]
        if self.recorder is not None:
            lines += [(178, 16, "%.1f 秒" % (ticks_diff(ticks_ms(), self.record_started) / 1000)),
                      (210, 16, "%d 帧" % self.recorder.count),
                      (242, 14, "%.1f FPS" % self.recorder.fps())]
        else:
            # Keep long errors in the console; split the short UI message.
            lines += [(178, 14, self.detail[:10]), (204, 14, self.detail[10:20])]
        for y, size, text in lines:
            self.screen.draw_string_advanced(x, y, size, text, color=color if y == 60 else (255, 255, 255))
        self.Display.show_image(self.screen)
        preview = None

    def run(self, max_frames=None, auto_record=False):
        raw = jpeg = None
        last_capture = last_preview = None
        seen = 0
        if auto_record:
            self.start_recording()
        while max_frames is None or seen < max_frames:
            exitpoint()
            if self.button.pressed():
                if self.recorder is not None:
                    self.finish_recording()
                else:
                    try:
                        self.start_recording()
                        last_capture = None
                    except Exception as exc:
                        self.status, self.detail, self.error = "开始失败", str(exc), True
                        print("REC_START_ERROR", type(exc).__name__, str(exc))
            now = ticks_ms()
            if self.recorder is not None and ticks_diff(now, self.record_started) >= MAX_RECORD_SECONDS * 1000:
                self.finish_recording("duration_limit")
            interval = max(1, 1000 // TARGET_FPS) if self.recorder is not None else PREVIEW_INTERVAL_MS
            if last_capture is not None and ticks_diff(now, last_capture) < interval:
                sleep_ms(5)
                continue
            last_capture = now
            raw = self.read_frame()
            if raw is None:
                sleep_ms(20)
                continue
            seen += 1
            captured = ticks_ms()
            if self.recorder is not None:
                try:
                    jpeg = self.encoder.encode(raw)
                    if not self.recorder.append(jpeg, captured):
                        self.finish_recording("size_limit")
                    elif ticks_diff(ticks_ms(), self.last_sync) >= SYNC_INTERVAL_MS:
                        self.recorder.checkpoint()
                        self.last_sync = ticks_ms()
                        free = free_megabytes(self.directory)
                        print("REC_PROGRESS", self.recorder.count, "frames", free, "MB free")
                        if free is not None and free < STOP_FREE_MB:
                            self.finish_recording("low_space")
                except Exception as exc:
                    print("REC_FRAME_ERROR", type(exc).__name__, str(exc))
                    self.finish_recording("encode_or_write_error")
                    self.status, self.detail, self.error = "录像已停止", str(exc), True
                finally:
                    jpeg = None
            if last_preview is None or ticks_diff(ticks_ms(), last_preview) >= PREVIEW_INTERVAL_MS:
                self.show(raw)
                last_preview = ticks_ms()
            raw = None
            gc.collect()

    def close(self):
        self.finish_recording("program_exit")
        for name in ("encoder", "sensor"):
            resource = getattr(self, name)
            setattr(self, name, None)
            if resource is not None:
                try:
                    resource.close() if name == "encoder" else resource.stop()
                except Exception as exc:
                    print("CLEANUP_ERROR", name, str(exc))
        self.screen = None
        gc.collect()
        if self.display_touched:
            try:
                self.Display.deinit()
            except Exception as exc:
                print("CLEANUP_ERROR display", str(exc))
            self.display_touched = False
        if self.media_touched:
            try:
                sleep_ms(100)
                self.Media.deinit()
            except Exception as exc:
                print("CLEANUP_ERROR media", str(exc))
            self.media_touched = False


def main(max_frames=None, auto_record=False, display=True, output_dir=None):
    """Defaults are manual-only. Bounded options are for board verification."""
    app = CaptureApp(display, output_dir)
    try:
        app.open()
        app.run(max_frames, auto_record)
    except KeyboardInterrupt:
        print("YOLO_CAPTURE_STOPPED")
    except BaseException as exc:
        if "IDE interrupt" not in str(exc):
            print("YOLO_CAPTURE_ERROR", type(exc).__name__, str(exc))
            raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
