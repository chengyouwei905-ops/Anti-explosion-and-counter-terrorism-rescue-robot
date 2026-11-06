"""Stationary AIM_SCAN only: native LAB ROI search, one hit, no pose fitting."""
import aim_config
from config import IMAGE_SIZE,COLOR_NAMES
from geometry import finite


class TargetScanDetector:
    def __init__(self,target_id,gate):
        if type(target_id) is not int or target_id not in (1,2,3):
            raise ValueError("INVALID_TARGET")
        self.target,self.gate=target_id,tuple(gate)
        if (len(self.gate)!=4 or any(type(v) is not int for v in self.gate) or
                min(self.gate[:2])<0 or min(self.gate[2:])<=0 or
                self.gate[0]+self.gate[2]>IMAGE_SIZE[0] or
                self.gate[1]+self.gate[3]>IMAGE_SIZE[1]):
            raise ValueError("INVALID_SCAN_GATE")
        self.ready=True

    def detect(self,raw):
        if not self.ready:
            return {"ok":False,"reason":"TARGET_DETECTOR_CLOSED","detections":[]}
        rgb=None
        try:
            if (raw.width(),raw.height())!=IMAGE_SIZE:
                raise ValueError("IMAGE_SIZE_MISMATCH")
            rgb=raw.to_rgb565()
            # Keep the field LAB values and original native size/stride limits.
            # A single-threshold code=1 denotes the requested color, not red.
            blobs=rgb.find_blobs((aim_config.LAB_THRESHOLDS[self.target-1],),
                                  roi=self.gate,pixels_threshold=200,area_threshold=900,
                                  merge=False,x_stride=2,y_stride=2)
            detections=[]
            x,y,w,h=self.gate
            for blob in blobs:
                if blob.code()!=1:
                    raise ValueError("TARGET_COLOR_AMBIGUOUS")
                box=tuple(blob.rect())
                if len(box)!=4 or not all(finite(v) for v in box) or min(box[2:])<=0:
                    raise ValueError("INVALID_DETECTION_BOX")
                if x<=box[0]+box[2]*.5<x+w and y<=box[1]+box[3]*.5<y+h:
                    detections.append({"color_id":self.target,"bbox_raw_px":box,
                                       "label":COLOR_NAMES[self.target]+"_target",
                                       "score":1.0,"score_kind":"lab_blob_only"})
            return {"ok":True,"reason":"OK","source":"color_target_scan","detections":detections}
        except Exception as exc:
            return {"ok":False,"reason":"TARGET_DETECTOR_ERROR:"+str(exc),"detections":[]}
        finally:
            rgb=None

    def close(self):
        self.ready=False


class TargetStoppedScanTask:
    def __init__(self,target_id,slot_id,gate):
        if type(slot_id) is not int or slot_id not in (1,2,3):
            raise ValueError("INVALID_SLOT_ID")
        self.detector=TargetScanDetector(target_id,gate)
        self.target,self.slot=target_id,slot_id
        self.gate=tuple(gate)
        self.min_hits=1
        self.mode="SCAN"
        self.last_frame=-1
        self.reset_window()

    def reset_window(self):
        self.hits=self.blank_hits=self.ambiguous=0
        self.last_reason="SCAN_WAIT_TARGET"

    def _state(self,reason,complete=False,found=False):
        self.last_reason=reason
        return {"ok":complete,"complete":complete,"reason":reason,"target_found":found,
                "target_slot":self.slot if found else None,"slot_id":self.slot,
                "slot_mapping":{self.target:self.slot} if found else None,
                "sample_count":0,"frame_id":self.last_frame}

    def poll(self,raw,frame_id,geometry_confirmed=False,stable=False):
        if self.mode!="SCAN":return self._state("TASK_NOT_ACTIVE")
        if type(frame_id) is not int or frame_id<=self.last_frame:
            return self._state("STALE_OR_DUPLICATE_FRAME")
        self.last_frame=frame_id
        if not geometry_confirmed or (raw.width(),raw.height())!=IMAGE_SIZE:
            self.reset_window()
            return self._state("RAW_GEOMETRY_NOT_CONFIRMED")
        if not stable:
            self.reset_window()
            return self._state("OBSERVATION_NOT_STABLE")
        result=self.detector.detect(raw)
        if not result.get("ok"):
            self.reset_window()
            return self._state(result.get("reason","VISION_ERROR"))
        detections=result["detections"]
        if not detections:
            self.hits=self.ambiguous=0
            self.blank_hits+=1
            return self._state("SCAN_WAIT_TARGET")
        self.blank_hits=0
        if len(detections)!=1:
            self.hits=0
            self.ambiguous+=1
            return self._state("SCAN_MULTIPLE_TARGETS_IN_GATE" if self.ambiguous>=3
                               else "SCAN_WAIT_SINGLE_TARGET")
        self.hits=1
        self.mode="DONE"
        return self._state("OK",complete=True,found=True)

    def finish_probe(self):
        if self.mode=="SCAN" and self.blank_hits>=1:
            self.mode="DONE"
            return self._state("SCAN_NOT_TARGET",complete=True)
        return self._state("TIMEOUT")

    def cancel(self):
        self.mode="CANCELLED"
        self.reset_window()

