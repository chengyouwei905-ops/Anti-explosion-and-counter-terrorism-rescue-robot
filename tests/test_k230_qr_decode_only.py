"""QR reads one task code, emits a header-only ACK, preserves other task logic."""
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
ROOT=Path(os.environ.get("K230_TEST_ROOT",str(Path(__file__).resolve().parents[1]/"output/k230_qr_decode_only_20261106/guanggongsai")))
PREVIOUS=Path(__file__).resolve().parents[1]/"output/k230_11am_aim_scan_only_20261106/guanggongsai"
os.environ["K230_TEST_ROOT"]=str(ROOT)
from test_k230_11am_aim_scan_only import AimScanOnlyTests,Factory,Detector,Link,Frame,p,cfg,TaskManager,SETTLING,RUNNING,REPORT
from tasks.task_qr_read import QRReadTask
import aim_config
import competition_main
import task_manager


class Code:
    def __init__(self,text):self.text=text
    def payload(self):return self.text
    def corners(self):raise AssertionError("QR read requested pose corners")


class Gray:
    def __init__(self,texts=("231",),size=(1920,1080),retry_texts=()):
        self.texts,self.size,self.retry_texts=texts,size,retry_texts
        self.calls=[];self.copied=None;self.enhancement=None
    def width(self):return self.size[0]
    def height(self):return self.size[1]
    def to_grayscale(self):return self
    def find_qrcodes(self,**kwargs):self.calls.append(kwargs);return [Code(text) for text in self.texts]
    def copy(self):self.copied=Gray(self.retry_texts,self.size);return self.copied
    def histeq(self,**kwargs):self.enhancement=kwargs


class OtherTaskRegression(AimScanOnlyTests):
    def test_all_original_parameters_and_unrelated_files_are_byte_identical(self):
        sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
        old={p.relative_to(PREVIOUS).as_posix():sha(p) for p in PREVIOUS.rglob('*') if p.is_file()}
        new={p.relative_to(ROOT).as_posix():sha(p) for p in ROOT.rglob('*') if p.is_file()}
        self.assertEqual(set(new)-set(old),{"tasks/task_qr_read.py"})
        self.assertEqual({name for name in old if old[name]!=new.get(name)},
                         {"aim_config.py","task_runner.py","task_manager.py","competition_main.py"})
        old_config=(PREVIOUS/"aim_config.py").read_bytes()
        self.assertEqual(old_config.replace(b"MAX_POINTS_PER_RING = 16",b"MAX_POINTS_PER_RING = 24",1),
                         (ROOT/"aim_config.py").read_bytes())
        self.assertEqual(aim_config.MAX_POINTS_PER_RING,24)


class QRReadTests(unittest.TestCase):
    def setUp(self):
        self.factory=Factory(Detector());self.factory.calibration=None
        self.link=Link();self.now=0;self.frame_id=0
        self.manager=TaskManager(self.link,self.factory,clock=lambda:self.now,diff=lambda a,b:a-b)

    def start(self,seq=42):
        self.manager.on_command(Frame(seq,p.QR_START,b"",self.now))
        self.assertEqual(self.manager.state,SETTLING)
        self.assertIsNone(self.manager.average)
        self.now+=cfg.SETTLE_MS;self.manager.tick()
        self.assertEqual(self.manager.state,RUNNING)

    def frame(self,gray=None):
        self.frame_id+=1
        self.manager.poll_frame({"image":Gray(size=(640,480)),"qr_image":gray or Gray(),
            "frame_id":self.frame_id,"epoch":0,"geometry_confirmed":False,"qr_geometry_confirmed":False,
            "capture_return_ms":self.now})

    def test_valid_code_completes_on_first_frame_without_pose_or_average(self):
        with patch.object(task_manager,"MeasurementAverage",side_effect=AssertionError("QR created an average")),\
             patch("tasks.task_qr.QRPoseEstimator",side_effect=AssertionError("QR created a pose estimator")):
            self.start();self.frame()
        self.assertEqual(self.manager.state,REPORT)
        self.assertEqual(self.manager.pending[2]["task"]["code"],"231")
        self.assertFalse(self.manager.pending[2]["pose_valid"])
        self.manager.flush_report()
        self.assertEqual(self.factory.selection["bomb_color_id"],2)
        self.assertEqual(self.factory.selection["target_color_id"],3)
        self.assertEqual(self.factory.selection["rescue_shape_id"],1)
        self.assertEqual(len(self.manager.last_report["payload"]),15)
        report=self.link.reports[-1]
        self.assertEqual((report["task_cmd"],report["input_seq"],report["status"],report["flags"],
                          report["slot_id"],report["sample_count"],report["reason_code"]),
                         (p.QR_START,42,p.SUCCESS,0,0,0,0))
        self.assertNotIn("position_tool_mm",report)

    def test_native_roi_matches_preview_and_high_resolution(self):
        task=QRReadTask(roi_size=cfg.QR_ROI_SIZE,roi_reference_size=cfg.QR_ROI_REFERENCE_SIZE)
        self.assertEqual(task.detection_roi((640,480)),(220,140,200,200))
        gray=Gray();result=task.measure(gray,1,geometry_confirmed=False,stable=True)
        self.assertTrue(result["ok"])
        self.assertEqual(gray.calls,[{"roi":(660,315,600,450)}])

    def test_contrast_retry_uses_same_roi_without_pose_checks(self):
        task=QRReadTask(contrast_retry=True,contrast_retry_every=1,
                        roi_size=(200,200),roi_reference_size=(640,480))
        gray=Gray((),retry_texts=("123",))
        result=task.measure(gray,1,False,True)
        self.assertTrue(result["ok"],result)
        self.assertEqual(result["task"]["code"],"123")
        self.assertEqual(gray.calls,gray.copied.calls)
        self.assertEqual(gray.copied.enhancement,{"adaptive":False})

    def test_invalid_task_codes_never_acknowledge_success(self):
        for code in ("124","12","1234"," 123","abc"):
            self.setUp();self.start();self.frame(Gray((code,)))
            self.assertEqual(self.manager.state,REPORT)
            self.assertEqual(self.manager.pending[0],p.FAILED)
            self.manager.flush_report();self.assertIsNone(self.factory.selection)

    def test_missing_or_multiple_codes_wait_for_unambiguous_content(self):
        self.start();self.frame(Gray(()))
        self.assertEqual(self.manager.state,RUNNING)
        self.frame(Gray(("123","231")))
        self.assertEqual(self.manager.state,RUNNING)
        self.assertEqual(self.manager.last_reason,"MULTIPLE_QRS_AMBIGUOUS")
        self.frame();self.assertEqual(self.manager.state,REPORT)

    def test_timeout_still_reports_failure_without_coordinates(self):
        self.start();self.now=self.manager.started_ms+self.manager.timeout_ms
        self.manager.tick();self.manager.flush_report()
        self.assertEqual(self.link.reports[-1]["reason_code"],p.REASONS["TIMEOUT"])
        self.assertIsNone(self.factory.selection)

    def test_pause_after_decode_requires_a_new_frame_on_resume(self):
        self.start();self.frame();self.assertEqual(self.manager.state,REPORT)
        self.manager.pause();self.assertIsNone(self.manager.pending)
        self.now+=100;self.manager.resume();self.now+=cfg.SETTLE_MS;self.manager.tick()
        self.frame(Gray(("123",)));self.manager.flush_report()
        self.assertEqual(self.factory.selection["code"],"123")
        self.assertEqual(len(self.link.reports),1)

    def test_stale_unstable_and_cancelled_reads_are_ignored(self):
        task=QRReadTask();gray=Gray()
        self.assertFalse(task.measure(gray,1,False,False)["ok"])
        self.assertFalse(task.measure(gray,1,False,True)["ok"])
        self.assertTrue(task.measure(gray,2,False,True)["ok"])
        task.cancel();self.assertFalse(task.measure(gray,3,False,True)["ok"])
        self.assertEqual(len(gray.calls),1)

    def test_qr_screen_has_one_decode_and_no_sample_window(self):
        self.start();seen=[]
        camera=SimpleNamespace(frame_id=1,render_debug=lambda image,metadata:seen.append(metadata))
        competition_main.debug_preview(camera,self.manager,None)
        self.assertIn("扫码:0/1",seen[-1]["lines"])
        self.assertIn("只读取任务码",seen[-1]["lines"])
        self.assertFalse(any("采样窗口" in line for line in seen[-1]["lines"]))
        self.frame();competition_main.debug_preview(camera,self.manager,None)
        self.assertIn("扫码:1/1",seen[-1]["lines"])

    def test_low_resolution_decode_and_preview_do_not_require_an_estimator(self):
        with patch.object(cfg,"QR_HIGH_RES_ENABLED",False):
            self.start();self.frame(Gray(size=(640,480)))
            seen=[];camera=SimpleNamespace(frame_id=1,render_debug=lambda image,metadata:seen.append(metadata))
            competition_main.debug_preview(camera,self.manager,None)
            self.assertEqual(seen[-1]["scan_gate"],(220,140,200,200))


if __name__=="__main__":
    suite=unittest.TestSuite((unittest.defaultTestLoader.loadTestsFromTestCase(OtherTaskRegression),
                              unittest.defaultTestLoader.loadTestsFromTestCase(QRReadTests)))
    with contextlib.redirect_stdout(io.StringIO()) as diagnostics:
        result=unittest.TextTestRunner(verbosity=2).run(suite)
    if not result.wasSuccessful():print(diagnostics.getvalue())
    sys.exit(not result.wasSuccessful())
