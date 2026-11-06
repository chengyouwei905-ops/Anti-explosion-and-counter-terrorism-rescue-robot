"""Stationary rescue identity scan: one matching YOLO frame, then refine later."""
from config import IMAGE_SIZE
from geometry import finite


class RescueStoppedScanTask:
    def __init__(self,detector,target_id,slot_id,gate):
        if type(target_id) is not int or target_id not in (1,2,3):
            raise ValueError("INVALID_TARGET")
        if type(slot_id) is not int or slot_id not in (1,2,3):
            raise ValueError("INVALID_SLOT_ID")
        self.detector,self.target,self.slot=detector,target_id,slot_id
        self.gate=tuple(gate)
        if (len(self.gate)!=4 or not all(finite(v) for v in self.gate) or
                min(self.gate[:2])<0 or min(self.gate[2:])<=0 or
                self.gate[0]+self.gate[2]>IMAGE_SIZE[0] or
                self.gate[1]+self.gate[3]>IMAGE_SIZE[1]):
            raise ValueError("INVALID_SCAN_GATE")
        self.mode="SCAN"
        self.last_frame=-1
        self.min_hits=1
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
        x,y,w,h=self.gate
        matches=[]
        for item in result["detections"]:
            identity=item.get("shape_id")
            box=item.get("bbox_raw_px",())
            if type(identity) is not int or identity not in (1,2,3):
                self.reset_window()
                return self._state("INVALID_TARGET")
            if len(box)!=4 or not all(finite(v) for v in box) or min(box[2:])<=0:
                self.reset_window()
                return self._state("INVALID_DETECTION_BOX")
            cx,cy=box[0]+box[2]*.5,box[1]+box[3]*.5
            if identity==self.target and x<=cx<x+w and y<=cy<y+h:
                matches.append(item)
        if not matches:
            self.hits=self.ambiguous=0
            self.blank_hits+=1
            return self._state("SCAN_WAIT_TARGET")
        self.blank_hits=0
        if len(matches)!=1:
            self.hits=0
            self.ambiguous+=1
            return self._state("SCAN_MULTIPLE_TARGETS_IN_GATE" if self.ambiguous>=3
                               else "SCAN_WAIT_SINGLE_TARGET")
        self.hits=1
        self.mode="DONE"
        return self._state("OK",complete=True,found=True)

    def finish_probe(self):
        # Only usable YOLO frames without the requested class may exhaust a stop.
        if self.mode=="SCAN" and self.blank_hits>=1:
            self.mode="DONE"
            return self._state("SCAN_NOT_TARGET",complete=True)
        return self._state("TIMEOUT")

    def cancel(self):
        self.mode="CANCELLED"
        self.reset_window()
