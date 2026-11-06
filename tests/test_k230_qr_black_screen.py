"""Run the real entrypoint through a QR wait >1s and its post-success idle loop."""
import contextlib
import io
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.dont_write_bytecode=True
ROOT=Path(os.environ.get("K230_TEST_ROOT",str(Path(__file__).resolve().parents[1]/"output/k230_qr_black_screen_fix_20261106/guanggongsai")))
os.environ["K230_TEST_ROOT"]=str(ROOT)
from test_k230_qr_decode_only import OtherTaskRegression,QRReadTests,Gray,Factory,Detector,Frame,p,cfg,TaskManager
import competition_main
import vision_link


class Clock:
    now=0
    def sleep(self,ms):self.now+=ms


class Link:
    def __init__(self,clock):
        self.clock=clock;self.events=[];self.queue=[];self.accept=False;self.closed=False
    def set_command_acceptance(self,value):
        self.accept=value
        if not value:self.queue=[]
    def send(self,cmd,payload):
        self.events.append((self.clock.now,cmd,bytes(payload)))
        if cmd==p.STATUS and payload==cfg.TOTAL_START_BYTES:
            self.queue.append(Frame(42,p.QR_START,b"",self.clock.now))
    def pop_priority(self):return None
    def take_overflow(self):return None
    def pop_command(self):return self.queue.pop(0) if self.accept and self.queue else None
    def healthy(self):return not self.closed
    def diagnostics(self):return {"healthy":not self.closed}
    def deinit(self):self.closed=True;return True


class Button:
    error=None
    def __init__(self):self.pending=True
    def healthy(self):return True
    def pressed(self):
        pressed,self.pending=self.pending,False
        return pressed
    def close(self):return True


class Camera:
    state="READY"
    epoch=0
    frame_id=0
    def __init__(self,clock,decode_at):
        self.clock,self.decode_at=clock,decode_at
        self.qr=False;self.previews=[];self.closed_at=None;self.frames_after_decode=0
    def set_qr_capture(self,enabled):
        if self.qr!=enabled:self.epoch+=1
        self.qr=enabled
    def get_frame(self):
        self.clock.now+=20;self.frame_id+=1
        if self.clock.now>=self.decode_at:self.frames_after_decode+=1
        frame={"image":Gray(size=(640,480)),"frame_id":self.frame_id,
               "epoch":self.epoch,"geometry_confirmed":True,"capture_return_ms":self.clock.now}
        if self.qr:
            frame["qr_image"]=Gray(("231",) if self.clock.now>=self.decode_at else ())
            frame["qr_geometry_confirmed"]=True
        return frame
    def render_debug(self,image,status):self.previews.append((self.clock.now,list(status["lines"])))
    def close(self):self.closed_at=self.clock.now;self.state="CLOSED"


class MainLoopTests(unittest.TestCase):
    def run_entrypoint(self,decode_at,loops=180):
        clock=Clock();clock.now=0
        link=Link(clock);camera=Camera(clock,decode_at);button=Button();factory=Factory(Detector())
        manager=TaskManager(link,factory,clock=lambda:clock.now,diff=lambda a,b:a-b)
        with patch.object(vision_link,"VisionLink",return_value=link),\
             patch.object(competition_main,"load_calibration",return_value=None),\
             patch.object(competition_main,"VisionTaskFactory",return_value=factory),\
             patch.object(competition_main,"CameraService",return_value=camera),\
             patch.object(competition_main,"TaskManager",return_value=manager),\
             patch.object(competition_main,"UserButton",return_value=button),\
             patch.object(competition_main,"ticks_ms",side_effect=lambda:clock.now),\
             patch.object(competition_main.os,"exitpoint",lambda:None,create=True),\
             patch.object(competition_main.time,"sleep_ms",clock.sleep,create=True):
            competition_main.main(max_loops=loops)
        return clock,link,camera,factory,manager

    def test_wait_over_one_second_then_decode_keeps_running_and_screen_alive(self):
        clock,link,camera,factory,manager=self.run_entrypoint(1750)
        results=[event for event in link.events if event[1]==p.RESULT]
        self.assertEqual(len(results),1)
        self.assertGreaterEqual(results[0][0],1750)
        report=p.decode_result(results[0][2])
        self.assertEqual((report["status"],report["input_seq"],report["flags"]),(p.SUCCESS,42,0))
        self.assertEqual(len(results[0][2]),15)
        self.assertGreater(clock.now,4000)
        self.assertGreater(camera.frames_after_decode,80)
        self.assertTrue(any("扫码已完成" in lines and now>2500 for now,lines in camera.previews))
        self.assertFalse(camera.qr)
        self.assertEqual(camera.closed_at,clock.now)
        stops=[event for event in link.events if event[1]==p.STATUS and event[2]==cfg.TOTAL_STOP_BYTES]
        self.assertEqual(len(stops),1)
        self.assertEqual(stops[0][0],clock.now) # Only explicit finite-test teardown stops.

    def test_missing_qr_can_wait_multiple_log_intervals_without_exception(self):
        clock,link,camera,factory,manager=self.run_entrypoint(100000)
        self.assertGreater(clock.now,4000)
        self.assertFalse(any(event[1]==p.RESULT for event in link.events))
        self.assertGreater(len(camera.previews),25)
        self.assertTrue(any("扫码:0/1" in lines and now>3000 for now,lines in camera.previews))

    def test_success_before_first_log_interval_and_later_idle_are_supported(self):
        clock,link,camera,factory,manager=self.run_entrypoint(300)
        results=[event for event in link.events if event[1]==p.RESULT]
        self.assertEqual(len(results),1)
        self.assertLess(results[0][0],1000)
        self.assertGreater(clock.now,4000)
        self.assertGreater(camera.frames_after_decode,140)


if __name__=="__main__":
    suite=unittest.TestSuite((unittest.defaultTestLoader.loadTestsFromTestCase(OtherTaskRegression),
                              unittest.defaultTestLoader.loadTestsFromTestCase(QRReadTests),
                              unittest.defaultTestLoader.loadTestsFromTestCase(MainLoopTests)))
    with contextlib.redirect_stdout(io.StringIO()) as diagnostics:
        result=unittest.TextTestRunner(verbosity=2).run(suite)
    if not result.wasSuccessful():print(diagnostics.getvalue())
    sys.exit(not result.wasSuccessful())
