"""Host regression of the deliverable K230 code; no KPU/camera hardware mocks leak into it."""
import ast
import contextlib
import io
from pathlib import Path
import sys
sys.dont_write_bytecode=True
import unittest
from types import SimpleNamespace
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]/"output/k230_stop_target_20261106/guanggongsai"
sys.path.insert(0,str(ROOT))
import protocol as p
import runtime_config as cfg
import task_runner
from task_manager import TaskManager,IDLE,SETTLING,RUNNING,REPORT
from task_runner import VisionTaskFactory
from tasks.task_qr import parse_task
from tasks.task_stop_scan import StoppedScanTask
from vision_link import Frame


class Raw:
    def width(self):return 640
    def height(self):return 480


class Detector:
    ready=True
    def __init__(self):self.items=[];self.reason=None;self.calls=0
    def detect(self,raw):
        self.calls+=1
        return {"ok":self.reason is None,"reason":self.reason or "OK","detections":self.items}
    def close(self):pass
    def show(self,identity,key="color_id",x=300):
        self.reason=None
        self.items=[{key:identity,"bbox_raw_px":(x,200,30,30)}]


class Factory(VisionTaskFactory):
    def __init__(self,detector):
        super().__init__(SimpleNamespace(camera=None),str(ROOT))
        self.fake_detector=detector
    def _model(self,kind):return self.fake_detector


class Link:
    def __init__(self):self.reports=[]
    def send(self,cmd,payload):
        assert cmd==p.RESULT
        self.reports.append(p.decode_result(payload))


class StopScanTests(unittest.TestCase):
    def setUp(self):
        self.detector=Detector();self.factory=Factory(self.detector)
        self.factory.completed({"cmd":p.QR_START},{"task":parse_task("231")})
        self.link=Link();self.now=0;self.frame_id=0
        self.manager=TaskManager(self.link,self.factory,clock=lambda:self.now,diff=lambda a,b:a-b)
        self.aim_patch=patch.object(task_runner,"ColorTargetDetector",lambda:self.detector)
        self.aim_patch.start();self.addCleanup(self.aim_patch.stop)

    def command(self,cmd=p.BOMB_SCAN,slot=1,seq=1,window=1000,target=0):
        frame=Frame(seq,cmd,bytes((target,slot,window&255,window>>8)),self.now)
        self.manager.on_command(frame)
        return frame

    def frame(self,epoch=0,captured=None):
        self.frame_id+=1
        self.manager.poll_frame({"image":Raw(),"frame_id":self.frame_id,"epoch":epoch,
            "capture_return_ms":self.now if captured is None else captured,"geometry_confirmed":True})

    def settle(self):
        self.now+=cfg.SETTLE_MS
        self.manager.tick()
        self.assertEqual(self.manager.state,RUNNING)

    def hit(self,identity,key="color_id"):
        self.detector.show(identity,key)
        for _ in range(3):self.frame();self.now+=20

    def test_qr_selection_all_kinds_and_stops(self):
        cases=((p.BOMB_SCAN,p.BOMB_MEASURE,2,"color_id","bomb"),
               (p.AIM_SCAN,p.AIM_MEASURE,3,"color_id","aim"),
               (p.RESCUE_SCAN,p.RESCUE_MEASURE,1,"shape_id","rescue"))
        for cmd,measure,target,key,kind in cases:
            for slot in (1,2,3):
                with self.subTest(cmd=cmd,slot=slot):
                    self.command(cmd,slot,seq=slot+cmd)
                    self.assertEqual(self.manager.request["target_id"],target)
                    self.assertEqual(self.manager.state,SETTLING)
                    self.frame();self.assertEqual(self.detector.calls,3*len(self.link.reports))
                    self.settle();self.hit(target,key)
                    self.assertEqual(self.manager.state,REPORT)
                    self.manager.flush_report()
                    result=self.link.reports[-1]
                    self.assertEqual((result["status"],result["slot_id"],result["reason_code"]),(p.SUCCESS,slot,0))
                    self.assertEqual(self.factory.maps[kind],{target:slot})
                    request=p.decode_command(Frame(slot+100,measure,b"",self.now))
                    job=self.factory.start(request)
                    self.assertEqual((request["target_id"],request["slot_id"]),(target,slot))
                    job.cancel()

    def test_non_target_is_explicit_terminal_and_next_request_works(self):
        self.command();self.settle();self.hit(1)
        self.manager.flush_report()
        result=self.link.reports[-1]
        self.assertEqual((result["status"],result["reason_code"],result["slot_id"]),(p.FAILED,20,1))
        self.assertIsNone(self.factory.maps["bomb"])
        calls=self.detector.calls
        self.frame();self.assertEqual(self.detector.calls,calls) # Moving while IDLE cannot count.
        self.command(slot=2,seq=2);self.settle();self.hit(2);self.manager.flush_report()
        self.assertEqual(self.link.reports[-1]["slot_id"],2)
        self.assertEqual(self.link.reports[-1]["status"],p.SUCCESS)

    def test_empty_position_expires_but_silence_is_a_fault(self):
        self.command();self.settle()
        for _ in range(3):self.frame();self.now+=20
        self.assertEqual(self.manager.state,RUNNING)
        self.now=self.manager.started_ms+self.manager.timeout_ms
        self.manager.tick();self.manager.flush_report()
        self.assertEqual(self.link.reports[-1]["reason_code"],20)
        self.command(slot=2,seq=2);self.settle()
        self.now=self.manager.started_ms+self.manager.timeout_ms
        self.manager.tick();self.manager.flush_report()
        self.assertEqual(self.link.reports[-1]["reason_code"],p.REASONS["TIMEOUT"])

    def test_missing_qr_and_model_error_are_not_misses(self):
        self.factory.selection=None
        self.command();self.manager.flush_report()
        self.assertEqual(self.link.reports[-1]["reason_code"],p.REASONS["TARGET_NOT_SELECTED"])
        self.factory.selection=parse_task("231")
        self.command(slot=2,seq=2);self.settle()
        self.detector.reason="MODEL_NOT_READY";self.frame();self.manager.flush_report()
        self.assertEqual(self.link.reports[-1]["reason_code"],9)

    def test_ambiguity_and_partial_hits_never_report_a_miss(self):
        self.command();self.settle();self.detector.show(1)
        self.detector.items.append({"color_id":2,"bbox_raw_px":(350,200,30,30)})
        for _ in range(3):self.frame()
        self.manager.flush_report()
        self.assertEqual(self.link.reports[-1]["reason_code"],p.REASONS["SCAN_AMBIGUOUS"])
        self.command(slot=2,seq=2);self.settle();self.detector.show(2);self.frame()
        self.now=self.manager.started_ms+self.manager.timeout_ms
        self.manager.tick();self.manager.flush_report()
        self.assertEqual(self.link.reports[-1]["reason_code"],p.REASONS["TIMEOUT"])

    def test_fresh_frames_dedup_pause_and_stop(self):
        command=self.command();self.detector.show(2)
        self.frame();self.assertEqual(self.detector.calls,0)
        self.settle();self.frame(captured=0);self.assertEqual(self.detector.calls,0)
        self.frame();calls=self.detector.calls
        self.manager.poll_frame({"image":Raw(),"frame_id":self.frame_id,"epoch":0,"geometry_confirmed":True})
        self.assertEqual(self.detector.calls,calls)
        self.manager.on_command(command);self.assertEqual(self.manager.job.task.hits,1)
        self.manager.pause();self.now+=100;self.manager.resume();self.settle()
        self.frame();self.frame();self.assertEqual(self.manager.state,RUNNING)
        self.frame();self.assertEqual(self.manager.state,REPORT)
        self.manager.on_command(Frame(99,p.STOP,b"",self.now))
        self.manager.flush_report()
        self.assertEqual(len(self.link.reports),1)
        self.assertEqual(self.link.reports[-1]["status"],p.CANCELLED)

    def test_camera_epoch_failure_and_total_stop_clear_selection(self):
        self.command();self.settle();self.frame();self.frame(epoch=1)
        self.manager.flush_report()
        self.assertEqual(self.link.reports[-1]["reason_code"],p.REASONS["CAMERA_ERROR"])
        self.command(slot=2,seq=2);self.manager.reset_session();self.manager.resume()
        self.assertIsNone(self.factory.selection)
        self.assertIsNone(self.manager.request)
        self.assertTrue(all(v is None for v in self.factory.maps.values()))

    def test_wire_payload_validation_and_legacy_scan(self):
        for payload in (b"\x00\x00\xe8\x03",b"\x00\x04\xe8\x03",
                        b"\x00\x01\x00\x00",b"\x00\x01\xff\xff",b"\x00\x01"):
            with self.assertRaises(ValueError):p.decode_command(Frame(1,p.BOMB_SCAN,payload,0))
        request=p.decode_command(Frame(1,p.BOMB_SCAN,b"",0))
        job=self.factory.start(request)
        self.assertFalse(job.stopped_scan)
        self.detector.show(2)
        for frame_id in range(3):result=job.poll({"image":Raw(),"frame_id":frame_id,"geometry_confirmed":True})
        self.assertFalse(result["complete"])

    def test_all_delivered_scripts_parse(self):
        for file in ROOT.rglob("*.py"):ast.parse(file.read_text(encoding="utf-8-sig"),filename=str(file))


if __name__=="__main__":
    # Keep production diagnostic prints available on failure, quiet on success.
    with contextlib.redirect_stdout(io.StringIO()) as diagnostics:
        result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(StopScanTests))
    if not result.wasSuccessful():print(diagnostics.getvalue())
    sys.exit(not result.wasSuccessful())
