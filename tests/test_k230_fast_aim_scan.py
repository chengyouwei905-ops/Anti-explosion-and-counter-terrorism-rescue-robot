"""Real factory/scan/manager/native-detector routing; no K230 hardware required."""
import contextlib
import io
import os
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch

sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[1]/"output/k230_fast_aim_scan_20261106/guanggongsai"
os.environ["K230_TEST_ROOT"]=str(ROOT)
from test_k230_scan_window import ScanWindowTests
from test_k230_stop_scan import Factory,Detector,Link,Frame,p,cfg,TaskManager,SETTLING,RUNNING,REPORT
from tasks.task_qr import parse_task
from tasks.task_stop_scan import StoppedScanTask
from target_color import ColorTargetDetector,extract_target_rings,_color_matches
import target_color
import aim_config
import competition_main


class Blob:
    def __init__(self,box=(290,210,60,60),code=1):self.box,self.bits=box,code
    def code(self):return self.bits
    def rect(self):return self.box


class RGB:
    def __init__(self,blobs=None):
        self.blobs=[Blob()] if blobs is None else blobs
        self.calls=[]
    def width(self):return 640
    def height(self):return 480
    def find_blobs(self,thresholds,**options):
        self.calls.append((tuple(thresholds),options))
        return self.blobs
    def get_pixel(self,*args,**kwargs):raise AssertionError("scan read Python pixels")
    def copy(self):raise AssertionError("scan allocated a contour mask")


class Raw:
    def __init__(self,rgb=None):self.rgb=rgb if rgb is not None else RGB()
    def width(self):return 640
    def height(self):return 480
    def to_rgb565(self):return self.rgb


class LabRing:
    def __init__(self):self.used=None
    def width(self):return 640
    def height(self):return 480
    def copy(self):self.mask=BinaryRing();return self.mask
    def binary(self,*args,**kwargs):raise AssertionError("original image modified")
    def get_pixel(self,*args,**kwargs):raise AssertionError("LAB path fell back to HSV/RGB")


class BinaryRing:
    def __init__(self):self.thresholds=None
    def binary(self,thresholds):self.thresholds=thresholds;return self
    def get_pixel(self,x,y,rgbtuple=True):
        radius=(x-320)**2+(y-240)**2
        return (255,255,255) if 22**2<=radius<=40**2 else (0,0,0)


class FallbackRing:
    def width(self):return 640
    def height(self):return 480
    def get_pixel(self,x,y,rgbtuple=True):
        radius=(x-320)**2+(y-240)**2
        # Inside the field's first LAB threshold, outside the old red HSV range.
        return (70,110,75) if 22**2<=radius<=40**2 else (0,0,0)


class FastAimTests(unittest.TestCase):
    def setUp(self):
        self.factory=Factory(Detector())
        self.factory.completed({"cmd":p.QR_START},{"task":parse_task("231")})
        self.now=0;self.link=Link();self.frame_id=0
        self.manager=TaskManager(self.link,self.factory,clock=lambda:self.now,diff=lambda a,b:a-b)

    def start(self,cmd=p.AIM_SCAN):
        payload=bytes((0,2,12000&255,12000>>8)) if cmd==p.AIM_SCAN else b""
        self.manager.on_command(Frame(1,cmd,payload,self.now))
        self.assertEqual(self.manager.state,SETTLING)
        self.now=cfg.SETTLE_MS;self.manager.tick()
        self.assertEqual(self.manager.state,RUNNING)

    def frame(self,raw=None,**extra):
        self.frame_id+=1
        frame={"image":raw or Raw(),"frame_id":self.frame_id,"epoch":0,
               "geometry_confirmed":True,"capture_return_ms":self.now}
        frame.update(extra);self.manager.poll_frame(frame)

    def test_one_native_roi_hit_reports_before_five_frame_average(self):
        self.start();job=self.manager.job
        self.assertEqual(job.task.min_hits,1)
        raw=Raw()
        with patch.object(target_color,"extract_target_rings",side_effect=AssertionError("scan extracted contours")),\
             patch.object(self.manager.average,"add",side_effect=AssertionError("scan averaged positions")):
            self.frame(raw)
        self.assertEqual(self.manager.state,REPORT)
        thresholds,options=raw.rgb.calls[0]
        self.assertEqual(thresholds,(aim_config.LAB_THRESHOLDS[2],))
        self.assertEqual(options["roi"],aim_config.SCAN_GATE_RAW_PX)
        self.assertEqual(len(raw.rgb.calls),1)
        self.assertEqual(job.task.hits,1)
        self.manager.flush_report()
        report=self.link.reports[-1]
        self.assertEqual((report["status"],report["slot_id"],report["sample_count"]),(p.SUCCESS,2,0))
        self.assertEqual(self.factory.maps["aim"],{3:2})

    def test_one_threshold_code_maps_to_requested_blue_not_red(self):
        detector=ColorTargetDetector(roi=aim_config.SCAN_GATE_RAW_PX,coarse_only=True,target_color=3)
        result=detector.detect(Raw())
        self.assertEqual(result["detections"][0]["color_id"],3)
        self.assertEqual(result["detections"][0]["score_kind"],"lab_blob_only")
        self.assertNotIn("contours",result["detections"][0])

    def test_empty_window_waits_then_misses_without_inventing_success(self):
        self.start();self.frame(Raw(RGB([])))
        self.assertEqual(self.manager.state,RUNNING)
        self.now=self.manager.started_ms+12000+cfg.SETTLE_MS
        self.manager.tick();self.manager.flush_report()
        report=self.link.reports[-1]
        self.assertEqual((report["status"],report["reason_code"]),(p.FAILED,20))

    def test_detector_errors_and_rejected_candidates_are_not_empty_misses(self):
        detector=SimpleNamespace(detect=lambda raw:{"ok":True,"detections":[],
            "rejected":[{"reason":"TARGET_COLOR_AMBIGUOUS"}]})
        task=StoppedScanTask(detector,3,2,"color_id",aim_config.SCAN_GATE_RAW_PX,1)
        result=task.poll(Raw(),1,True,True)
        self.assertEqual(result["reason"],"TARGET_COLOR_AMBIGUOUS")
        self.assertFalse(task.finish_probe()["complete"])
        self.assertEqual(task.blank_hits,0)

    def test_multiple_targets_are_not_accepted_as_a_single_target(self):
        self.start();self.frame(Raw(RGB([Blob(),Blob((345,210,40,40))])))
        self.assertEqual(self.manager.state,RUNNING)
        self.assertFalse(self.link.reports)
        self.assertEqual(self.manager.last_reason,"SCAN_WAIT_SINGLE_TARGET")

    def test_stale_unstable_and_wrong_size_frames_cannot_hit(self):
        detector=ColorTargetDetector(roi=aim_config.SCAN_GATE_RAW_PX,coarse_only=True,target_color=3)
        task=StoppedScanTask(detector,3,2,"color_id",aim_config.SCAN_GATE_RAW_PX,1)
        self.assertFalse(task.poll(Raw(),1,True,False)["complete"])
        self.assertFalse(task.poll(Raw(),1,True,True)["complete"])
        self.assertFalse(task.poll(Raw(),2,False,True)["complete"])
        self.assertTrue(task.poll(Raw(),3,True,True)["target_found"])

    def test_native_lab_mask_drives_inner_and_outer_ring_extraction(self):
        rgb=LabRing()
        result=extract_target_rings(rgb,(280,200,80,80),3)
        self.assertTrue(result["ok"],result)
        self.assertEqual(rgb.mask.thresholds,[aim_config.LAB_THRESHOLDS[2]])
        self.assertGreaterEqual(len(result["outer_points_px"]),24)
        self.assertGreaterEqual(len(result["inner_points_px"]),20)
        self.assertTrue(all(270<x<370 and 190<y<290 for x,y in result["outer_points_px"]))

    def test_host_fallback_matches_lab_instead_of_old_hsv(self):
        self.assertTrue(_color_matches((70,110,75),1))
        result=extract_target_rings(FallbackRing(),(280,200,80,80),1)
        self.assertTrue(result["ok"],result)

    def test_precision_filters_roi_before_contour_and_candidate_limits(self):
        outside=[Blob((10+i,10,20,20)) for i in range(12)]
        detector=ColorTargetDetector(target_color=3)
        raw=Raw(RGB(outside+[Blob()]))
        with patch.object(target_color,"extract_target_rings",return_value={"ok":True}) as extract:
            result=detector.detect(raw)
        self.assertTrue(result["ok"],result)
        self.assertEqual(extract.call_count,1)
        self.assertEqual(extract.call_args.args[1],(290,210,60,60))

    def test_scan_does_not_remove_precision_five_frame_requirement(self):
        self.factory.maps["aim"]={3:2}
        self.start(p.AIM_MEASURE)
        def measured(raw,frame_id,**kwargs):
            return {"ok":True,"reason":"OK","frame_id":frame_id,"position_tool_mm":[0,300,100],
                    "coordinate_frame":"E_commanded_gripper_center","units":"mm",
                    "reference_point":"target_physical_center","control_ready":True,"handeye_accepted":True}
        with patch.object(self.manager.job.task,"poll",side_effect=measured):
            for _ in range(4):
                self.frame();self.now+=20
                self.assertEqual(self.manager.state,RUNNING)
            self.frame()
        self.assertEqual(self.manager.state,REPORT)
        self.manager.flush_report()
        self.assertEqual(self.link.reports[-1]["sample_count"],5)

    def test_scan_display_uses_identity_count_not_position_average(self):
        self.start();seen=[]
        camera=SimpleNamespace(frame_id=0,render_debug=lambda image,metadata:seen.append(metadata))
        competition_main.debug_preview(camera,self.manager,None)
        self.assertIn("识别:0/1",seen[-1]["lines"])
        self.assertNotIn("采样:0/5",seen[-1]["lines"])
        self.assertIn("目标:蓝靶",seen[-1]["lines"])

    def test_scan_display_handles_missing_qr_selection(self):
        self.factory.selection=None
        self.manager.on_command(Frame(1,p.AIM_SCAN,bytes((0,2,12000&255,12000>>8)),self.now))
        seen=[]
        camera=SimpleNamespace(frame_id=0,render_debug=lambda image,metadata:seen.append(metadata))
        competition_main.debug_preview(camera,self.manager,None)
        self.assertIn("目标:未选择",seen[-1]["lines"])

    def test_native_scan_accepts_unchanged_field_aim_config(self):
        fields=("STOPPED_SCAN_MIN_HITS","TARGET_MIN_COLOR_PIXELS","TARGET_MIN_BBOX_AREA_PX")
        saved={name:getattr(aim_config,name) for name in fields}
        try:
            for name in fields:delattr(aim_config,name)
            self.start();self.frame()
            self.assertEqual(self.manager.state,REPORT)
        finally:
            for name,value in saved.items():setattr(aim_config,name,value)


if __name__=="__main__":
    suite=unittest.TestSuite((unittest.defaultTestLoader.loadTestsFromTestCase(ScanWindowTests),
                              unittest.defaultTestLoader.loadTestsFromTestCase(FastAimTests)))
    with contextlib.redirect_stdout(io.StringIO()) as diagnostics:
        result=unittest.TextTestRunner(verbosity=2).run(suite)
    if not result.wasSuccessful():print(diagnostics.getvalue())
    sys.exit(not result.wasSuccessful())
