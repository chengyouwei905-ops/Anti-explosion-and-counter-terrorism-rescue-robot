"""Rescue YOLO identity selection on one frame; refinement remains separate."""
import contextlib
import hashlib
import io
import json
import os
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch

sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[1]/"output/k230_rescue_yolo_single_hit_20261106/guanggongsai"
BASE=Path(__file__).resolve().parents[1]/"output/k230_qr_black_screen_fix_20261106/guanggongsai"
os.environ["K230_TEST_ROOT"]=str(ROOT)
from test_k230_qr_black_screen import OtherTaskRegression,QRReadTests,MainLoopTests
from test_k230_11am_aim_scan_only import Factory,Detector,Link,Frame,p,cfg,TaskManager,SETTLING,RUNNING,REPORT,Raw
from tasks.task_qr import parse_task
from tasks.task_rescue_stop_scan import RescueStoppedScanTask
from tasks.task_rescue_object import RescueObjectTask
from rescue_detector import RescueYoloDetector
from geometry import CameraModel
import rescue_config
import rescue_object_pose
import competition_main


class ExistingTaskRegression(OtherTaskRegression):
    def test_all_original_parameters_and_unrelated_files_are_byte_identical(self):
        sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
        old={p.relative_to(BASE).as_posix():sha(p) for p in BASE.rglob('*') if p.is_file()}
        new={p.relative_to(ROOT).as_posix():sha(p) for p in ROOT.rglob('*') if p.is_file()}
        self.assertEqual(set(new)-set(old),{"tasks/task_rescue_stop_scan.py"})
        self.assertEqual({name for name in old if old[name]!=new.get(name)},
                         {"task_runner.py","competition_main.py"})
        for name in ("aim_config.py","rescue_config.py","runtime_config.py","config.py",
                     "models/rescue_three_shapes.template.json","models/rescue_best.kmodel",
                     "tasks/task_rescue_object.py","rescue_object_pose.py","rescue_detector.py","yolo_detector.py"):
            self.assertEqual(old[name],new[name],name)

    def test_bomb_and_rescue_still_require_their_existing_three_hits(self):
        # Bomb retains three hits; the explicitly requested rescue path now uses one.
        for cmd,target,key,hits in ((p.BOMB_SCAN,2,"color_id",3),(p.RESCUE_SCAN,1,"shape_id",1)):
            self.setUp();job=self.command(cmd)
            self.assertEqual(job.task.min_hits,hits)
            self.detector.show(target,key);self.settle()
            for _ in range(hits-1):
                self.frame();self.assertEqual(self.manager.state,RUNNING)
            self.frame();self.assertEqual(self.manager.state,REPORT)


class NormalizedDetector(Detector):
    def detect(self,raw):
        result=super().detect(raw);result["source"]="yolo_rescue";return result


class TestBackend:
    is_test_backend=True
    def __init__(self,items):self.items=items;self.calls=0
    def load(self,metadata):return {"ok":True}
    def infer(self,raw):self.calls+=1;return {"space":"raw_pixels","detections":self.items}
    def close(self):pass


class RescueScanTests(unittest.TestCase):
    def setUp(self):
        self.detector=NormalizedDetector();self.factory=Factory(self.detector)
        self.factory.completed({"cmd":p.QR_START},{"task":parse_task("231")})
        self.link=Link();self.now=0;self.frame_id=0
        self.manager=TaskManager(self.link,self.factory,clock=lambda:self.now,diff=lambda a,b:a-b)

    def start(self,slot=2,target=0,seq=1):
        self.manager.on_command(Frame(seq,p.RESCUE_SCAN,bytes((target,slot,12000&255,12000>>8)),self.now))
        self.assertEqual(self.manager.state,SETTLING)
        self.now+=cfg.SETTLE_MS;self.manager.tick()
        self.assertIsInstance(self.manager.job.task,RescueStoppedScanTask)
        self.assertEqual(self.manager.state,RUNNING)

    def frame(self,**extra):
        self.frame_id+=1
        frame={"image":Raw(),"frame_id":self.frame_id,"epoch":0,
               "capture_return_ms":self.now,"geometry_confirmed":True}
        frame.update(extra);self.manager.poll_frame(frame)

    def test_single_yolo_hit_completes_without_estimating_pose_or_averaging(self):
        with patch.object(rescue_object_pose.RescueObjectEstimator,"__init__",side_effect=AssertionError("scan created pose estimator")):
            self.start();self.detector.show(1,"shape_id")
            with patch.object(self.manager.average,"add",side_effect=AssertionError("scan averaged positions")):
                self.frame()
        self.assertEqual(self.detector.calls,1)
        self.assertEqual(self.manager.state,REPORT)
        self.manager.flush_report();report=self.link.reports[-1]
        self.assertEqual((report["status"],report["slot_id"],report["sample_count"]),(p.SUCCESS,2,0))
        self.assertEqual(self.factory.maps["rescue"],{1:2})
        self.assertNotIn("position_tool_mm",report)

    def test_each_requested_shape_and_physical_slot(self):
        for shape in (1,2,3):
            for slot in (1,2,3):
                self.start(slot,shape,seq=shape*10+slot);self.detector.show(shape,"shape_id")
                self.frame();self.manager.flush_report()
                self.assertEqual(self.link.reports[-1]["slot_id"],slot)
                self.assertEqual(self.factory.maps["rescue"],{shape:slot})

    def test_target_wins_when_other_classes_are_in_the_same_roi(self):
        self.start(target=2)
        self.detector.items=[{"shape_id":shape,"bbox_raw_px":(x,200,40,60)}
                             for shape,x in ((1,180),(2,300),(3,440))]
        self.frame();self.manager.flush_report()
        self.assertEqual(self.link.reports[-1]["status"],p.SUCCESS)
        self.assertEqual(self.factory.maps["rescue"],{2:2})

    def test_other_shapes_do_not_end_the_window_early(self):
        self.start(target=2);self.detector.show(1,"shape_id")
        for _ in range(4):self.frame();self.assertEqual(self.manager.state,RUNNING)
        self.detector.show(2,"shape_id");self.frame()
        self.assertEqual(self.manager.state,REPORT)

    def test_roi_is_applied_to_yolo_box_centers(self):
        self.start();self.detector.show(1,"shape_id",x=20);self.frame()
        self.assertEqual(self.manager.state,RUNNING)
        self.detector.show(1,"shape_id",x=120);self.frame()
        self.assertEqual(self.manager.state,REPORT)

    def test_zero_or_nonmatching_detections_use_the_full_window(self):
        for items in ([],[{"shape_id":2,"bbox_raw_px":(300,200,30,30)}]):
            self.setUp();self.start();self.detector.items=items;self.frame()
            self.now=self.manager.started_ms+10000+cfg.SETTLE_MS
            self.manager.tick();self.assertEqual(self.manager.state,RUNNING)
            self.now=self.manager.started_ms+12000+cfg.SETTLE_MS
            self.manager.tick();self.manager.flush_report()
            self.assertEqual(self.link.reports[-1]["reason_code"],20)

    def test_duplicate_targets_are_not_randomly_selected(self):
        self.start();self.detector.items=[{"shape_id":1,"bbox_raw_px":(x,200,30,40)} for x in (200,400)]
        for _ in range(3):self.frame()
        self.manager.flush_report()
        self.assertEqual(self.link.reports[-1]["reason_code"],p.REASONS["SCAN_AMBIGUOUS"])

    def test_model_error_is_not_a_non_target_result(self):
        self.start();self.detector.reason="MODEL_NOT_READY";self.frame()
        self.manager.flush_report()
        self.assertEqual(self.link.reports[-1]["reason_code"],9)
        self.assertIsNone(self.factory.maps["rescue"])

    def test_inference_error_after_blank_frame_does_not_expire_as_miss(self):
        self.start();self.frame()
        self.detector.reason="DETECTION_ERROR:temporary";self.frame()
        self.now=self.manager.started_ms+self.manager.timeout_ms
        self.manager.tick();self.manager.flush_report()
        self.assertEqual(self.link.reports[-1]["reason_code"],p.REASONS["TIMEOUT"])

    def test_old_unstable_and_unconfirmed_frames_cannot_match(self):
        self.detector.show(1,"shape_id")
        task=RescueStoppedScanTask(self.detector,1,2,rescue_config.SCAN_GATE_RAW_PX)
        self.assertFalse(task.poll(Raw(),1,True,False)["complete"])
        self.assertFalse(task.poll(Raw(),1,True,True)["complete"])
        self.assertFalse(task.poll(Raw(),2,False,True)["complete"])
        self.assertEqual(self.detector.calls,0)
        self.assertTrue(task.poll(Raw(),3,True,True)["target_found"])

    def test_pause_resets_partial_scan_and_resume_accepts_one_fresh_match(self):
        self.start();self.frame();self.manager.pause();self.now+=100;self.manager.resume()
        self.now+=cfg.SETTLE_MS;self.manager.tick();self.detector.show(1,"shape_id");self.frame()
        self.assertEqual(self.manager.state,REPORT)

    def test_unchanged_yolo_adapter_keeps_confidence_filter_and_class_mapping(self):
        metadata=json.loads((ROOT/"models/rescue_three_shapes.template.json").read_text(encoding="utf-8"))
        self.assertEqual(metadata["confidence_threshold"],.55)
        metadata["model_path"]=str(ROOT/metadata["model_path"])
        # Native YOLO postprocessing uses metadata .55; the unchanged adapter
        # separately has the field's .45 floor. This stub exercises the adapter.
        backend=TestBackend([{"class_id":0,"score":.44,"bbox_xywh":[300,200,30,40]},
                             {"class_id":2,"score":.8,"bbox_xywh":[400,200,30,40]}])
        detector=RescueYoloDetector(metadata,backend,allow_test_backend=True)
        self.assertTrue(detector.load()["ok"])
        result=detector.detect(Raw())
        self.assertEqual([item["shape_id"] for item in result["detections"]],[3])
        task=RescueStoppedScanTask(detector,3,2,rescue_config.SCAN_GATE_RAW_PX)
        self.assertTrue(task.poll(Raw(),1,True,True)["target_found"])
        self.assertEqual(backend.calls,2)

    def test_hit_only_selects_target_then_real_bbox_refinement_still_needs_five_frames(self):
        camera=CameraModel([[500,0,320],[0,500,240],[0,0,1]],[0,0,0,0,0])
        self.factory.calibration=SimpleNamespace(camera=camera,handeye_accepted=True,
            point_to_tool=lambda point,allow:{"ok":True,"reason":"OK","control_ready":True,"position_tool_mm":list(point)})
        self.start();self.detector.items=[{"shape_id":1,"bbox_raw_px":(295,215,50,50)}]
        self.frame();self.manager.flush_report()
        self.manager.on_command(Frame(2,p.RESCUE_MEASURE,b"",self.now))
        self.assertIsInstance(self.manager.job.task,RescueObjectTask)
        self.now+=cfg.SETTLE_MS;self.manager.tick()
        for _ in range(4):self.frame();self.assertEqual(self.manager.state,RUNNING)
        self.frame();self.assertEqual(self.manager.state,REPORT)
        self.manager.flush_report();report=self.link.reports[-1]
        self.assertEqual((report["task_cmd"],report["status"],report["sample_count"]),(p.RESCUE_MEASURE,p.SUCCESS,5))
        self.assertAlmostEqual(report["position_tool_mm"][2],500)

    def test_rescue_scan_screen_shows_identity_one_frame(self):
        self.start();seen=[]
        camera=SimpleNamespace(frame_id=1,render_debug=lambda image,metadata:seen.append(metadata))
        competition_main.debug_preview(camera,self.manager,None)
        self.assertIn("识别:0/1",seen[-1]["lines"])
        self.assertIn("目标:圆柱",seen[-1]["lines"])


if __name__=="__main__":
    suite=unittest.TestSuite((unittest.defaultTestLoader.loadTestsFromTestCase(ExistingTaskRegression),
                              unittest.defaultTestLoader.loadTestsFromTestCase(QRReadTests),
                              unittest.defaultTestLoader.loadTestsFromTestCase(MainLoopTests),
                              unittest.defaultTestLoader.loadTestsFromTestCase(RescueScanTests)))
    with contextlib.redirect_stdout(io.StringIO()) as diagnostics:
        result=unittest.TextTestRunner(verbosity=2).run(suite)
    if not result.wasSuccessful():print(diagnostics.getvalue())
    sys.exit(not result.wasSuccessful())
