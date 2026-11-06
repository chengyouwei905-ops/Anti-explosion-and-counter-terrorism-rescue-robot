"""One stationary scan position, numbered by the MCU rather than object order."""
import config
from geometry import finite


class StoppedScanTask:
    def __init__(self,detector,target_id,slot_id,identity_key,gate,min_hits):
        if type(target_id) is not int or target_id not in (1,2,3):
            raise ValueError("INVALID_TARGET")
        if type(slot_id) is not int or slot_id not in (1,2,3):
            raise ValueError("INVALID_SLOT_ID")
        self.detector,self.target,self.slot=detector,target_id,slot_id
        self.identity_key,self.gate,self.min_hits=identity_key,tuple(gate),min_hits
        if (len(self.gate)!=4 or not all(finite(v) for v in self.gate) or
                min(self.gate[2:])<=0 or type(min_hits) is not int or min_hits<1):
            raise ValueError("INVALID_SCAN_STABILITY")
        self.mode="SCAN"
        self.last_frame=-1
        self.reset_window()

    def reset_window(self):
        self.candidate=self.center=None
        self.hits=self.blank_hits=self.ambiguous=0
        self.last_reason="SCAN_WAIT_TARGET"

    def _state(self,reason,complete=False,found=False):
        self.last_reason=reason
        return {"ok":complete,"complete":complete,"reason":reason,
                "target_found":found,"target_slot":self.slot if found else None,
                "slot_id":self.slot,"slot_mapping":{self.target:self.slot} if found else None,
                "sample_count":0,"frame_id":self.last_frame}

    def poll(self,raw,frame_id,geometry_confirmed=False,stable=False):
        if self.mode!="SCAN":return self._state("TASK_NOT_ACTIVE")
        if type(frame_id) is not int or frame_id<=self.last_frame:
            return self._state("STALE_OR_DUPLICATE_FRAME")
        self.last_frame=frame_id
        if not geometry_confirmed or (raw.width(),raw.height())!=config.IMAGE_SIZE:
            self.reset_window()
            return self._state("RAW_GEOMETRY_NOT_CONFIRMED")
        if not stable:
            self.reset_window()
            return self._state("OBSERVATION_NOT_STABLE")
        detection=self.detector.detect(raw)
        if not detection.get("ok"):
            self.reset_window()
            return self._state(detection.get("reason","VISION_ERROR"))
        x,y,w,h=self.gate
        eligible=[]
        for item in detection["detections"]:
            bx,by,bw,bh=item["bbox_raw_px"]
            if not all(finite(v) for v in (bx,by,bw,bh)) or min(bw,bh)<=0:
                self.reset_window()
                return self._state("INVALID_DETECTION_BOX")
            cx,cy=bx+bw*.5,by+bh*.5
            if x<=cx<x+w and y<=cy<y+h:
                identity=item[self.identity_key]
                if type(identity) is not int or identity not in (1,2,3):
                    self.reset_window()
                    return self._state("INVALID_TARGET")
                eligible.append((identity,cx,cy))
        if not eligible:
            if detection.get("rejected"):
                self.reset_window()
                return self._state(detection["rejected"][0].get("reason","VISION_ERROR"))
            self.candidate=self.center=None
            self.hits=self.ambiguous=0
            self.blank_hits+=1
            return self._state("SCAN_WAIT_TARGET")
        self.blank_hits=0
        if len(eligible)!=1:
            self.candidate=self.center=None
            self.hits=0
            self.ambiguous+=1
            return self._state("SCAN_MULTIPLE_TARGETS_IN_GATE" if self.ambiguous>=3
                               else "SCAN_WAIT_SINGLE_TARGET")
        self.ambiguous=0
        identity,cx,cy=eligible[0]
        close=(self.center is not None and
               (cx-self.center[0])**2+(cy-self.center[1])**2<=config.SCAN_MAX_STEP_PX**2)
        self.hits=self.hits+1 if self.candidate==identity and close else 1
        self.candidate,self.center=identity,(cx,cy)
        if self.hits<self.min_hits:return self._state("SCAN_STABILIZING")
        self.mode="DONE"
        return self._state("OK" if identity==self.target else "SCAN_NOT_TARGET",
                           complete=True,found=identity==self.target)

    def finish_probe(self):
        # Empty but usable frames can exhaust this position. Inference/camera
        # errors, ambiguity and incomplete identity hits must never mean "miss".
        if self.mode=="SCAN" and self.blank_hits>=self.min_hits:
            self.mode="DONE"
            return self._state("SCAN_NOT_TARGET",complete=True)
        return self._state("TIMEOUT")

    def cancel(self):
        self.mode="CANCELLED"
        self.reset_window()
