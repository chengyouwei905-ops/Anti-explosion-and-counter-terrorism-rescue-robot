"""QR ROI routing, preview mapping and original-image pose coordinates."""
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
import competition_main
from qr_pose import CAMERA_MATRIX,DIST_COEFFS
from tasks.task_qr import QRTask
from task_runner import VisionTaskFactory,VisionJob
from vision_debug import DetectionTrace


class Gray:
    def __init__(self,size=(1920,1080),enhanced=False):
        self.size,self.enhanced=size,enhanced
        self.calls=[];self.copied=None;self.enhancement=None
    def width(self):return self.size[0]
    def height(self):return self.size[1]
    def find_qrcodes(self,**kwargs):self.calls.append(kwargs);return []
    def copy(self):self.copied=Gray(self.size,True);return self.copied
    def histeq(self,**kwargs):self.enhancement=kwargs


def calibration():
    return SimpleNamespace(camera=SimpleNamespace(K=CAMERA_MATRIX,D=DIST_COEFFS,image_size=(640,480)),
        handeye_accepted=True,point_to_tool=lambda point,allow:{
            "position_tool_mm":list(point),"ok":True,"control_ready":True,"reason":"OK"})


def task(**kwargs):
    return QRTask(calibration(),high_resolution_size=(1920,1080),roi_size=(200,200),**kwargs)


class QrRoiTests(unittest.TestCase):
    def test_center_in_actual_resolution_and_preview_reference(self):
        qr=task()
        self.assertEqual(qr.detection_roi((1920,1080)),(860,440,200,200))
        self.assertEqual(qr.detection_roi((640,480)),(220,140,200,200))
        qr=task(roi_reference_size=(640,480))
        self.assertEqual(qr.detection_roi((1920,1080)),(660,315,600,450))
        self.assertEqual(qr.detection_roi((640,480)),(220,140,200,200))

    def test_raw_and_enhancement_share_roi_without_cropping(self):
        qr=task(contrast_retry=True,contrast_retry_every=1)
        gray=Gray();seen=[]
        def detect(image,roi=None,geometry_confirmed=False):
            seen.append((image.size,roi,geometry_confirmed,image.enhanced))
            return []
        qr.high_resolution_estimator=SimpleNamespace(image_size=(1920,1080),detect=detect)
        result=qr.measure(gray,1,geometry_confirmed=True,stable=True)
        self.assertEqual(result["reason"],"QR_NOT_FOUND")
        self.assertEqual(seen,[((1920,1080),(860,440,200,200),True,False),
                               ((1920,1080),(860,440,200,200),True,True)])
        self.assertEqual(gray.copied.enhancement,{"adaptive":False})

    def test_actual_estimator_passes_original_corners_to_solver(self):
        qr=task()
        corners=((900,480),(1020,480),(1020,600),(900,600))
        class Code:
            def payload(self):return "231"
            def corners(self):return corners
        gray=Gray()
        def find(**kwargs):
            gray.calls.append(kwargs)
            return [Code()]
        gray.find_qrcodes=find
        solved=[]
        def estimate(points):
            solved.append(points)
            return {"ok":True,"reason":"ok","position_camera_mm":[1,2,1300],
                    "position_3sigma_mm":1,"center_disagreement_mm":1,"reprojection_error_px":.1}
        qr.high_resolution_estimator.estimate_from_corners=estimate
        result=qr.measure(gray,1,geometry_confirmed=True,stable=True)
        self.assertTrue(result["ok"])
        self.assertEqual(solved,[corners])
        self.assertEqual(gray.calls,[{"roi":(860,440,200,200)}])
        self.assertEqual(result["position_tool_mm"],[1,2,1300])
        self.assertEqual(result["task"]["code"],"231")

    def test_preview_detection_uses_same_roi(self):
        qr=task(contrast_retry=True,contrast_retry_every=1)
        job=VisionJob(None,{"cmd":p.QR_START,"target_id":None},qr)
        job.debug_trace=DetectionTrace("qr",None)
        gray=Gray()
        job.preview({"image":gray,"qr_image":gray,"frame_id":1})
        self.assertEqual(gray.calls,[{"roi":(860,440,200,200)}])
        self.assertEqual(gray.copied.calls,[{"roi":(860,440,200,200)}])
        self.assertIsNone(job.debug_trace.error)

    def test_factory_configuration_and_preview_box_mapping(self):
        factory=VisionTaskFactory(calibration(),str(ROOT))
        for reference,raw_roi,preview_roi in (
                (None,(860,440,200,200),(287,196,67,89)),
                ((640,480),(660,315,600,450),(220,140,200,200))):
            with patch.object(cfg,"QR_ROI_SIZE",(200,200)),patch.object(cfg,"QR_ROI_REFERENCE_SIZE",reference):
                job=factory.start(p.decode_command(SimpleNamespace(cmd=p.QR_START,seq=1,payload=b"")))
                self.assertEqual(job.task.detection_roi((1920,1080)),raw_roi)
                manager=SimpleNamespace(state="RUNNING",request=job.request,average=SimpleNamespace(samples=[]),
                    last_reason="QR_NOT_FOUND",last_report=None,factory=factory,job=job)
                status=[]
                camera=SimpleNamespace(frame_id=1,render_debug=lambda image,data:status.append(data))
                competition_main.debug_preview(camera,manager,None,Gray((640,480)))
                self.assertEqual(status[-1]["scan_gate"],preview_roi)

    def test_invalid_sizes_and_disabled_roi(self):
        for size in ((0,200),(-1,200),(200.0,200),(200,), (1000,1000)):
            with self.subTest(size=size),self.assertRaises(ValueError):QRTask(calibration(),roi_size=size)
        with self.assertRaises(ValueError):task(roi_reference_size=(0,480))
        qr=QRTask(calibration(),roi_size=None)
        self.assertIsNone(qr.detection_roi((640,480)))
        gray=Gray((640,480));result=qr.measure(gray,1,geometry_confirmed=True,stable=True)
        self.assertEqual(gray.calls,[{}])
        self.assertEqual(result["reason"],"QR_NOT_FOUND")


if __name__=="__main__":unittest.main(verbosity=2)
