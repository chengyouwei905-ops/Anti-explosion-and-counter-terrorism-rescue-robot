"""Single owner of the calibrated Sensor/Display/MediaManager, main thread only."""
import gc
import time
import runtime_config as cfg


def ticks_ms():
    method=getattr(time,"ticks_ms",None)
    return method() if method is not None else int(time.monotonic()*1000)


def ticks_diff(a,b):
    method=getattr(time,"ticks_diff",None)
    return method(a,b) if method is not None else a-b


def settle_outputs():
    """Allow display/DMA teardown to settle before releasing image/VB memory."""
    method=getattr(time,"sleep_ms",None)
    if method is not None:method(100)
    else:time.sleep(0.1)


def lcd_wrap(text,columns=18):
    """Bound panel width, treating CJK characters as two Latin columns."""
    lines=[];line="";width=0
    for char in str(text):
        size=2 if ord(char)>127 else 1
        if char=="\n" or width+size>columns:
            lines.append(line);line="";width=0
            if char=="\n":continue
        line+=char;width+=size
    if line or not lines:lines.append(line)
    return lines


class CameraService:
    def __init__(self,sensor_type=None,display=None,media=None,channel=0,clock=ticks_ms,diff=ticks_diff,image_factory=None,qr_enabled=False,display_to_ide=None):
        self.sensor_type,self.display,self.media,self.channel=sensor_type,display,media,channel
        self.clock,self.diff=clock,diff
        self.sensor=None
        self.display_touched=self.media_touched=False
        self.state="CLOSED"
        self.error=self.display_error=None
        self.epoch=self.frame_id=self.failures=self.restarts=0
        self.warmup=0
        self.restart_at=0
        self.image_factory=image_factory
        self._qr_notice_visible=False
        self.screen=None
        self.display_frames=0
        self.qr_enabled=bool(qr_enabled)
        self.qr_requested=False
        self.display_to_ide=display_to_ide

    def set_qr_capture(self,enabled):
        requested=bool(enabled) and self.qr_enabled
        if requested!=self.qr_requested:
            self.qr_requested=requested
            self.epoch+=1

    def _load_types(self):
        if self.sensor_type is None:
            from media.sensor import Sensor
            from media.display import Display
            from media.media import MediaManager,CAM_CHN_ID_0
            self.sensor_type,self.display,self.media,self.channel=Sensor,Display,MediaManager,CAM_CHN_ID_0

    def _create_sensor(self):
        self.sensor=self.sensor_type(id=0)
        self.sensor.reset()
        self.sensor.set_framesize(self.sensor_type.VGA,chn=self.channel)
        self.sensor.set_pixformat(self.sensor_type.RGB888,chn=self.channel)
        if self.qr_enabled:
            if cfg.QR_CAPTURE_CHANNEL==self.channel:
                raise ValueError("QR_CHANNEL_MUST_DIFFER_FROM_RGB_CHANNEL")
            width,height=cfg.QR_CAPTURE_SIZE
            # 同一传感器/acq_win。原VGA默认crop=None，此路显式不裁剪。
            self.sensor.set_framesize(width=width,height=height,
                                      chn=cfg.QR_CAPTURE_CHANNEL,crop=False)
            self.sensor.set_pixformat(self.sensor_type.GRAYSCALE,chn=cfg.QR_CAPTURE_CHANNEL)

    def _ensure_display(self):
        if not cfg.DISPLAY_ENABLED:return False
        self._load_types()
        if not self.display_touched:
            # The official display-and-touch example supports LCD-only output.
            # No Sensor instance or snapshots are needed before the user key.
            self.display_touched=True
            try:
                self.display.init(self.display.ST7701,width=cfg.DISPLAY_WIDTH,
                                  height=cfg.DISPLAY_HEIGHT,
                                  to_ide=cfg.DISPLAY_TO_IDE if self.display_to_ide is None else bool(self.display_to_ide))
            except BaseException:
                try:self.display.deinit()
                finally:
                    self.display_touched=False
                    settle_outputs()
                raise
        return True

    def open(self):
        if self.state!="CLOSED": return
        self._load_types()
        try:
            self._create_sensor()
            self._ensure_display()
            self.media_touched=True
            self.media.init()
            self.sensor.run()
            self.epoch+=1
            self.warmup=cfg.CAMERA_WARMUP_FRAMES
            self.state="WARMUP" if self.warmup else "READY"
        except BaseException:
            self.close()
            raise

    def _stop_sensor(self):
        old,self.sensor=self.sensor,None
        try:
            if old is not None: old.stop()
        finally:
            old=None
            gc.collect()

    def get_frame(self):
        if self.state=="ERROR": return None
        operation=self.state
        try:
            if self.state=="CLOSED":
                self.open()
                return None
            if self.state=="RECOVER_WAIT":
                if self.diff(self.clock(),self.restart_at)<cfg.CAMERA_RESTART_WAIT_MS: return None
                self._create_sensor()
                self.sensor.run()
                self.warmup=cfg.CAMERA_WARMUP_FRAMES
                self.state="WARMUP" if self.warmup else "READY"
                return None
            qr_raw=None
            if self.qr_requested:
                qr_raw=self.sensor.snapshot(chn=cfg.QR_CAPTURE_CHANNEL,
                                            timeout=cfg.SNAPSHOT_TIMEOUT_MS,auto_reboot=False)
                if (qr_raw.width(),qr_raw.height())!=tuple(cfg.QR_CAPTURE_SIZE):
                    raise RuntimeError("QR_CAPTURE_SIZE_MISMATCH")
            raw=self.sensor.snapshot(chn=self.channel,timeout=cfg.SNAPSHOT_TIMEOUT_MS,auto_reboot=False)
            returned=self.clock()
            self.frame_id+=1
            self.failures=0
            if self.state=="WARMUP":
                self.warmup-=1
                if self.warmup<=0: self.state="READY"
                return None
            frame={"image":raw,"frame_id":self.frame_id,"capture_return_ms":returned,
                   "epoch":self.epoch,"geometry_confirmed":cfg.RAW_GEOMETRY_CONFIRMED}
            if qr_raw is not None:
                frame["qr_image"]=qr_raw
                frame["qr_geometry_confirmed"]=cfg.RAW_GEOMETRY_CONFIRMED
            return frame
        except (RuntimeError,OSError) as exc:
            self.error="CAMERA_ERROR:"+str(exc)
            self.epoch+=1
            if operation in ("CLOSED","RECOVER_WAIT"):
                try: self._stop_sensor()
                except Exception: pass
                self.state="ERROR"
                return None
            self.failures+=1
            if self.failures>=cfg.SNAPSHOT_ATTEMPTS:
                if self.restarts>=cfg.MAX_CAMERA_RESTARTS:
                    self.state="ERROR"
                    return None
                try:
                    self._stop_sensor()
                except Exception as stop_error:
                    self.error="CAMERA_ERROR:"+str(stop_error)
                    self.state="ERROR"
                    return None
                self.restarts+=1
                self.failures=0
                self.restart_at=self.clock()
                self.state="RECOVER_WAIT"
            return None

    def render_debug(self,image=None,status=None):
        preview=None
        try:
            if not self._ensure_display():return
            if self.screen is None:
                if self.image_factory is None:
                    import image as image_module
                    self.screen=image_module.Image(cfg.DISPLAY_WIDTH,cfg.DISPLAY_HEIGHT,image_module.RGB565)
                else:self.screen=self.image_factory(cfg.DISPLAY_WIDTH,cfg.DISPLAY_HEIGHT)
                self.screen.clear()
                self.screen.draw_string_advanced(190,220,24,"等待启动",color=(210,220,235))
            if image is not None:
                # Only this display copy is annotated. The calibrated 640x480
                # raw image remains unchanged for inference and geometry.
                preview=image.to_rgb565()
                self.screen.draw_image(preview,0,0)
                gate=(status or {}).get("scan_gate")
                if gate is not None:self.screen.draw_rectangle(*gate,color=(255,220,0),thickness=2)
                detections=(status or {}).get("detections")
                if detections:
                    from vision_debug import draw_detections
                    draw_detections(self.screen,detections,(status or {}).get("source_size",(640,480)),
                                    (status or {}).get("pose_valid"))
            elif (status or {}).get("clear_preview"):
                self.screen.draw_rectangle(0,0,640,480,color=(0,0,0),fill=True)
            notice=(status or {}).get("notice")
            # v1.8-75 supports these existing drawing calls; no new OSD/font API.
            if notice:
                self.screen.draw_rectangle(0,384,640,96,color=(28,18,20),fill=True)
                color=notice.get("color",(255,190,60))
                self.screen.draw_string_advanced(8,388,24,notice["title"],color=color)
                for y,key in ((418,"code"),(446,"detail")):
                    parts=lcd_wrap(notice.get(key,""),74)
                    text=parts[0]+("..." if len(parts)>1 else "")
                    self.screen.draw_string_advanced(8,y,16,text,color=color)
            elif self._qr_notice_visible and image is None:
                self.screen.draw_rectangle(0,384,640,96,color=(0,0,0),fill=True)
            self._qr_notice_visible=bool(notice)
            self.screen.draw_rectangle(640,0,cfg.DISPLAY_WIDTH-640,cfg.DISPLAY_HEIGHT,
                                       color=(18,24,35),fill=True)
            y=8
            for text in (status or {}).get("lines",["K230 视觉","实时预览"]):
                for part in lcd_wrap(text):
                    if y>430:break
                    self.screen.draw_string_advanced(648,y,16,part,color=(235,240,250))
                    y+=20
            self.screen.draw_string_advanced(648,454,16,"用户键启/停",color=(255,220,80))
            self.display.show_image(self.screen)
            self.display_frames+=1
            self.display_error=None
        except Exception as exc:
            error=str(exc)
            if error!=self.display_error:print("K230_DISPLAY_ERROR",error)
            self.display_error=error
        finally: preview=None

    def show(self,image):
        self.render_debug(image)

    def close(self):
        errors=[]
        had_output=self.display_touched or self.media_touched
        try:
            self._stop_sensor()
        except Exception as exc: errors.append(str(exc))
        try:
            if self.display_touched: self.display.deinit()
        except Exception as exc: errors.append(str(exc))
        finally: self.display_touched=False
        # Keep the rendered canvas alive until display/DMA shutdown has settled,
        # then release its references before destroying the media buffers.
        if had_output:settle_outputs()
        self.screen=None
        gc.collect()
        try:
            if self.media_touched: self.media.deinit()
        except Exception as exc: errors.append(str(exc))
        finally:
            self.media_touched=False
            self.state="CLOSED"
        if errors: self.error="CAMERA_CLOSE_ERROR:"+";".join(errors)
