"""11:00 backup: isolated native-ROI single-frame AIM_SCAN, parameters intact."""
import ast
import contextlib
import hashlib
import io
import os
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch

sys.dont_write_bytecode=True
ROOT=Path(os.environ.get("K230_TEST_ROOT",str(Path(__file__).resolve().parents[1]/"output/k230_11am_aim_scan_only_20261106/guanggongsai")))
BASE=Path(r"C:\Users\26617\OneDrive\Desktop\广工赛\11月6日11点k230备份\guanggongsai")
os.environ["K230_TEST_ROOT"]=str(ROOT)
from test_k230_stop_scan import Factory,Detector,Link,Frame,p,cfg,TaskManager,SETTLING,RUNNING,REPORT
from tasks.task_qr import parse_task
from tasks.task_target_stop_scan import TargetStoppedScanTask,TargetScanDetector
from tasks.task_stop_scan import StoppedScanTask
import aim_config
import competition_main
import target_color
import target_pose


class Blob:
    def __init__(self,box=(290,210,60,60),code=1):self.box,self.bits=box,code
    def code(self):return self.bits
    def rect(self):return self.box


class RGB:
    def __init__(self,blobs=None,error=None):
        self.blobs=[Blob()] if blobs is None else blobs
        self.error=error;self.calls=[]
    def find_blobs(self,thresholds,**kwargs):
        self.calls.append((tuple(thresholds),kwargs))
        if self.error:raise ValueError(self.error)
        return self.blobs
    def get_pixel(self,*args,**kwargs):raise AssertionError("scan iterated pixels")
    def copy(self,*args,**kwargs):raise AssertionError("scan copied a contour mask")


class Raw:
    def __init__(self,rgb=None):self.rgb=rgb if rgb is not None else RGB()
    def width(self):return 640
    def height(self):return 480
    def to_rgb565(self):return self.rgb


class AimScanOnlyTests(unittest.TestCase):
    def setUp(self):
        self.detector=Detector();self.factory=Factory(self.detector)
        self.factory.completed({"cmd":p.QR_START},{"task":parse_task("231")})
        self.now=0;self.frame_id=0;self.link=Link()
        self.manager=TaskManager(self.link,self.factory,clock=lambda:self.now,diff=lambda a,b:a-b)

    def command(self,cmd=p.AIM_SCAN,slot=2,target=0,seq=1):
        payload=bytes((target,slot,12000&255,12000>>8)) if cmd in p.SCAN_COMMANDS else b""
        self.manager.on_command(Frame(seq,cmd,payload,self.now))
        return self.manager.job

    def settle(self):
        self.now+=cfg.SETTLE_MS;self.manager.tick()
        self.assertEqual(self.manager.state,RUNNING)

    def frame(self,raw=None,**extra):
        self.frame_id+=1
        frame={"image":raw or Raw(),"frame_id":self.frame_id,"epoch":0,
               "geometry_confirmed":True,"capture_return_ms":self.now}
        frame.update(extra);self.manager.poll_frame(frame)

    def test_all_original_parameters_and_unrelated_files_are_byte_identical(self):
        sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
        base={p.relative_to(BASE).as_posix():sha(p) for p in BASE.rglob('*') if p.is_file()}
        delivery={p.relative_to(ROOT).as_posix():sha(p) for p in ROOT.rglob('*') if p.is_file()}
        self.assertEqual(len(base),55)
        self.assertEqual(set(delivery)-set(base),{"tasks/task_target_stop_scan.py"})
        self.assertFalse(set(base)-set(delivery))
        changed={name for name in base if base[name]!=delivery[name]}
        self.assertEqual(changed,{"task_runner.py","competition_main.py"})
        for name in ("aim_config.py","runtime_config.py","config.py","rescue_config.py",
                     "protocol.py","target_color.py","target_pose.py","tasks/task_aim.py",
                     "tasks/task_stop_scan.py","task_manager.py"):
            self.assertEqual(base[name],delivery[name],name)

    def test_every_delivered_script_parses(self):
        for path in ROOT.rglob("*.py"):
            ast.parse(path.read_text(encoding="utf-8-sig"),filename=str(path))

    def test_scan_one_native_roi_hit_without_contours_pose_or_average(self):
        job=self.command();self.assertIsInstance(job.task,TargetStoppedScanTask)
        self.assertEqual(job.task.min_hits,1)
        self.settle();raw=Raw()
        with patch.object(target_color,"extract_target_rings",side_effect=AssertionError("scan extracted rings")),\
             patch.object(target_pose,"fit_target_center",side_effect=AssertionError("scan solved pose")),\
             patch.object(self.manager.average,"add",side_effect=AssertionError("scan averaged positions")):
            self.frame(raw)
        self.assertEqual(self.manager.state,REPORT)
        thresholds,kwargs=raw.rgb.calls[0]
        self.assertEqual(thresholds,(aim_config.LAB_THRESHOLDS[2],))
        self.assertEqual(kwargs["roi"],aim_config.SCAN_GATE_RAW_PX)
        self.assertEqual((kwargs["pixels_threshold"],kwargs["area_threshold"]),(200,900))
        self.assertEqual(len(raw.rgb.calls),1)
        self.manager.flush_report();report=self.link.reports[-1]
        self.assertEqual((report["status"],report["slot_id"],report["sample_count"]),(p.SUCCESS,2,0))
        self.assertEqual(self.factory.maps["aim"],{3:2})
        self.assertNotIn("position_tool_mm",report)

    def test_single_threshold_code_one_maps_to_each_requested_color(self):
        for color in (1,2,3):
            raw=Raw()
            detector=TargetScanDetector(color,aim_config.SCAN_GATE_RAW_PX)
            result=detector.detect(raw)
            self.assertTrue(result["ok"],result)
            self.assertEqual(result["detections"][0]["color_id"],color)
            self.assertEqual(raw.rgb.calls[0][0],(aim_config.LAB_THRESHOLDS[color-1],))

    def test_all_slots_return_the_physical_slot_on_first_hit(self):
        for slot in (1,2,3):
            self.command(slot=slot,seq=slot);self.settle();self.frame()
            self.manager.flush_report()
            self.assertEqual(self.link.reports[-1]["slot_id"],slot)

    def test_empty_frame_waits_full_twelve_seconds(self):
        self.command();self.settle();self.frame(Raw(RGB([])))
        self.now=self.manager.started_ms+10000+cfg.SETTLE_MS
        self.manager.tick();self.assertEqual(self.manager.state,RUNNING)
        self.now=self.manager.started_ms+12000+cfg.SETTLE_MS
        self.manager.tick();self.manager.flush_report()
        report=self.link.reports[-1]
        self.assertEqual((report["status"],report["reason_code"]),(p.FAILED,20))

    def test_silence_and_detector_errors_do_not_become_empty_misses(self):
        for raw in (None,Raw(RGB(error="BROKEN_FRAME"))):
            self.setUp();self.command();self.settle()
            if raw is not None:self.frame(raw)
            self.now=self.manager.started_ms+self.manager.timeout_ms
            self.manager.tick();self.manager.flush_report()
            self.assertEqual(self.link.reports[-1]["reason_code"],p.REASONS["TIMEOUT"])

    def test_ambiguity_never_succeeds_or_expires_as_empty(self):
        self.command();self.settle()
        for _ in range(3):self.frame(Raw(RGB([Blob(),Blob((350,210,30,40))])))
        self.assertEqual(self.manager.state,REPORT)
        self.manager.flush_report()
        self.assertEqual(self.link.reports[-1]["reason_code"],p.REASONS["SCAN_AMBIGUOUS"])

    def test_unstable_old_and_unconfirmed_frames_cannot_hit(self):
        task=TargetStoppedScanTask(3,2,aim_config.SCAN_GATE_RAW_PX)
        raw=Raw()
        self.assertFalse(task.poll(raw,1,True,False)["complete"])
        self.assertFalse(task.poll(raw,1,True,True)["complete"])
        self.assertFalse(task.poll(raw,2,False,True)["complete"])
        self.assertFalse(raw.rgb.calls)
        self.assertTrue(task.poll(raw,3,True,True)["target_found"])

    def test_frame_captured_before_settle_is_not_used(self):
        self.command();self.settle();raw=Raw()
        self.frame(raw,capture_return_ms=0)
        self.assertEqual(self.manager.state,RUNNING)
        self.assertFalse(raw.rgb.calls)
        self.frame(raw);self.assertEqual(self.manager.state,REPORT)

    def test_wrong_image_dimensions_are_rejected(self):
        task=TargetStoppedScanTask(3,2,aim_config.SCAN_GATE_RAW_PX)
        raw=Raw();raw.width=lambda:320
        result=task.poll(raw,1,True,True)
        self.assertEqual(result["reason"],"RAW_GEOMETRY_NOT_CONFIRMED")
        self.assertFalse(task.finish_probe()["complete"])

    def test_invalid_gate_and_identity_are_rejected(self):
        for gate in ((0,0,0,100),(-1,0,50,50),(600,0,100,50),(1.5,0,50,50)):
            with self.assertRaises(ValueError):TargetScanDetector(1,gate)
        for identity in (0,4,True):
            with self.assertRaises(ValueError):TargetScanDetector(identity,aim_config.SCAN_GATE_RAW_PX)

    def test_raw_coordinates_are_not_shifted_by_roi_twice(self):
        raw=Raw();detector=TargetScanDetector(3,aim_config.SCAN_GATE_RAW_PX)
        self.assertEqual(detector.detect(raw)["detections"][0]["bbox_raw_px"],(290,210,60,60))

    def test_bomb_and_rescue_still_require_their_existing_three_hits(self):
        for cmd,target,key in ((p.BOMB_SCAN,2,"color_id"),(p.RESCUE_SCAN,1,"shape_id")):
            self.setUp();job=self.command(cmd)
            self.assertIsInstance(job.task,StoppedScanTask)
            self.assertEqual(job.task.min_hits,3)
            self.detector.show(target,key);self.settle()
            for _ in range(2):
                self.frame();self.assertEqual(self.manager.state,RUNNING)
            self.frame();self.assertEqual(self.manager.state,REPORT)

    def test_target_precision_still_uses_original_five_frame_average(self):
        self.factory.maps["aim"]={3:2}
        self.command(p.AIM_MEASURE);self.settle()
        def measured(raw,frame_id,**kwargs):
            return {"ok":True,"reason":"OK","frame_id":frame_id,"position_tool_mm":[0,300,100],
                    "coordinate_frame":"E_commanded_gripper_center","units":"mm",
                    "reference_point":"target_physical_center","control_ready":True,"handeye_accepted":True}
        with patch.object(self.manager.job.task,"poll",side_effect=measured):
            for _ in range(4):self.frame();self.assertEqual(self.manager.state,RUNNING)
            self.frame()
        self.manager.flush_report()
        self.assertEqual(self.link.reports[-1]["sample_count"],5)

    def test_aim_scan_screen_uses_one_hit_and_other_scan_screen_is_unchanged(self):
        self.command();seen=[]
        camera=SimpleNamespace(frame_id=0,render_debug=lambda image,metadata:seen.append(metadata))
        competition_main.debug_preview(camera,self.manager,None)
        self.assertIn("识别:0/1",seen[-1]["lines"])
        self.assertIn("目标:蓝靶",seen[-1]["lines"])
        self.setUp();self.command(p.BOMB_SCAN)
        competition_main.debug_preview(camera,self.manager,None)
        self.assertIn("采样:0/5",seen[-1]["lines"])

    def test_pause_and_new_scan_restart_with_fresh_samples(self):
        self.command();self.settle();self.frame(Raw(RGB([])))
        self.manager.pause();self.now+=100;self.manager.resume();self.settle()
        self.frame();self.assertEqual(self.manager.state,REPORT)
        self.manager.flush_report()
        self.command(slot=3,seq=2);self.settle();self.frame()
        self.manager.flush_report()
        self.assertEqual([item["slot_id"] for item in self.link.reports],[2,3])


if __name__=="__main__":
    with contextlib.redirect_stdout(io.StringIO()) as diagnostics:
        result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(AimScanOnlyTests))
    if not result.wasSuccessful():print(diagnostics.getvalue())
    sys.exit(not result.wasSuccessful())
