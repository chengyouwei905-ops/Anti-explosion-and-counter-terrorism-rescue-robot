"""Run the stop-scan suite and longer-window checks against the 09:00-based delivery."""
import os
from pathlib import Path
import unittest

ROOT=Path(os.environ.get("K230_TEST_ROOT",str(Path(__file__).resolve().parents[1]/"output/k230_scan_window_20261106/guanggongsai")))
os.environ["K230_TEST_ROOT"]=str(ROOT)
from test_k230_stop_scan import StopScanTests,Frame,p,cfg,RUNNING,REPORT


class ScanWindowTests(StopScanTests):
    def test_scan_window_wire_limits(self):
        for cmd in p.SCAN_COMMANDS:
            for slot in (1,2,3):
                for window in (250,5000,10000,12000,60000):
                    payload=bytes((0,slot,window&255,window>>8))
                    request=p.decode_command(Frame(1,cmd,payload,0))
                    self.assertEqual(request["scan_window_ms"],window)
                    self.assertEqual(request["slot_id"],slot)
                for window in (0,249,60001,65535):
                    with self.assertRaises(ValueError):
                        p.decode_command(Frame(1,cmd,bytes((0,slot,window&255,window>>8)),0))

    def test_twelve_second_empty_scan_uses_full_window(self):
        self.command(window=12000);self.settle()
        for _ in range(3):self.frame();self.now+=20
        self.now=self.manager.started_ms+10000+cfg.SETTLE_MS
        self.manager.tick()
        self.assertEqual(self.manager.state,RUNNING)
        self.assertFalse(self.link.reports)
        self.now=self.manager.started_ms+12000+cfg.SETTLE_MS-1
        self.manager.tick();self.assertEqual(self.manager.state,RUNNING)
        self.now+=1;self.manager.tick();self.manager.flush_report()
        self.assertEqual(self.link.reports[-1]["reason_code"],20)

    def test_twelve_second_window_still_exits_early_on_target(self):
        self.command(window=12000);self.settle();self.hit(2)
        self.assertEqual(self.manager.state,REPORT)
        self.assertLess(self.now-self.manager.started_ms,12000)
        self.manager.flush_report()
        self.assertEqual(self.link.reports[-1]["status"],p.SUCCESS)

    def test_latest_tuning_is_preserved(self):
        self.assertEqual(cfg.MEASUREMENT_SAMPLES,5)
        self.assertEqual(cfg.MAX_POSITION_SCATTER_MM,20.0)
        self.assertEqual(cfg.QR_ROI_SIZE,(200,200))
        self.assertEqual(cfg.QR_ROI_REFERENCE_SIZE,(640,480))


if __name__=="__main__":
    unittest.main(defaultTest="ScanWindowTests",verbosity=2)
