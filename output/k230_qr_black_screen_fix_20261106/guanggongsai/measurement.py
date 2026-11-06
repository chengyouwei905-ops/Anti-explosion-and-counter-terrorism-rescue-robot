"""Fresh-frame position averaging in the commanded gripper frame, mm."""
from geometry import finite, norm


def position_fields(observation):
    """Remove orientation exports at the shared observation boundary."""
    result=dict(observation)
    for key in ("T_tool_from_qr","T_camera_from_qr","quaternion_tool_from_qr",
                "normal_tool","normal_camera","normal_valid","axis_camera"):
        result.pop(key,None)
    return result


class MeasurementAverage:
    def __init__(self,count=5,max_scatter_mm=8.0):
        if type(count) is not int or not 1<=count<=32: raise ValueError("INVALID_SAMPLE_COUNT")
        if not finite(max_scatter_mm) or max_scatter_mm<=0: raise ValueError("INVALID_SCATTER_LIMIT")
        self.count,self.max_scatter=count,max_scatter_mm
        self.samples=[]
        self.last_frame=-1
        self.identity=None

    def reset(self):
        self.samples=[]
        self.identity=None

    def add(self,result):
        frame=result.get("frame_id")
        if type(frame) is not int or frame<0:
            self.reset(); return None,"INVALID_MEASUREMENT_FRAME"
        if frame<=self.last_frame: return None,"STALE_OR_DUPLICATE_FRAME"
        self.last_frame=frame
        point=result.get("position_tool_mm")
        if not result.get("ok") or point is None:
            self.reset(); return None,result.get("reason","INVALID_MEASUREMENT")
        if len(point)!=3 or not all(finite(v) for v in point):
            self.reset(); return None,"NONFINITE_MEASUREMENT"
        if result.get("coordinate_frame")!="E_commanded_gripper_center" or result.get("units")!="mm":
            self.reset(); return None,"INVALID_MEASUREMENT_COORDINATES"
        task=result.get("task")
        identity=(result.get("target_id"),result.get("slot_id"),result.get("reference_point"),
                  result.get("coordinate_frame"),task.get("code") if task else None)
        if identity!=self.identity: self.reset(); self.identity=identity
        sample=position_fields(result)
        sample["position_tool_mm"]=list(point)
        self.samples.append(sample)
        mean=[sum(s["position_tool_mm"][i] for s in self.samples)/len(self.samples) for i in range(3)]
        scatter=max(norm([s["position_tool_mm"][i]-mean[i] for i in range(3)]) for s in self.samples)
        if scatter>self.max_scatter:
            self.samples=[sample]; return None,"POSITION_NOT_STABLE"
        if len(self.samples)<self.count: return None,"MEASUREMENT_STABILIZING"
        output=dict(sample)
        output.update(position_tool_mm=mean,scatter_mm=scatter,sample_count=len(self.samples),
                      sample_frame_ids=[s["frame_id"] for s in self.samples],
                      control_ready=all(s.get("control_ready") is True for s in self.samples),
                      handeye_accepted=all(s.get("handeye_accepted") is True for s in self.samples))
        vectors=[s.get("grasp_position_tool_mm") for s in self.samples]
        output["grasp_position_tool_mm"]=None
        if all(v is not None and len(v)==3 and all(finite(x) for x in v) for v in vectors):
            average=[sum(v[i] for v in vectors)/len(vectors) for i in range(3)]
            if max(norm([v[i]-average[i] for i in range(3)]) for v in vectors)<=self.max_scatter:
                output["grasp_position_tool_mm"]=average
        return output,"OK"


class QRMeasurementAverage(MeasurementAverage):
    """Average only valid QR poses in a bounded window, tolerating brief misses."""

    def __init__(self,count,max_scatter_mm,window_ms,clock,diff):
        if type(window_ms) is not int or window_ms<=0:
            raise ValueError("INVALID_QR_SAMPLE_WINDOW")
        super().__init__(count,max_scatter_mm)
        self.window_ms,self.clock,self.diff=window_ms,clock,diff

    def add(self,result,captured_ms=None):
        now=self.clock()
        self.samples=[sample for sample in self.samples
                      if 0<=self.diff(now,sample["_qr_capture_ms"])<=self.window_ms]
        if not self.samples:self.identity=None
        frame=result.get("frame_id")
        if type(frame) is not int or frame<0 or frame<=self.last_frame:
            return super().add(result)
        reason=result.get("reason")
        # These observations contribute no pose. Only an identified SAME code
        # may bridge an uncertain position; multiple/changed/invalid codes reset.
        same_code=(self.samples and result.get("task") is not None and
                   result["task"].get("code")==self.samples[-1].get("task",{}).get("code"))
        if (not result.get("ok") and
                (reason=="QR_NOT_FOUND" or
                 (same_code and reason in ("qr_center_ambiguous","qr_position_uncertain")))):
            self.last_frame=frame
            return None,reason
        capture=now if captured_ms is None else captured_ms
        if type(capture) is not int or not 0<=self.diff(now,capture)<=self.window_ms:
            self.last_frame=frame
            self.reset()
            return None,"QR_SAMPLE_EXPIRED"
        sample=dict(result)
        sample["_qr_capture_ms"]=capture
        output,reason=super().add(sample)
        if output is not None:output.pop("_qr_capture_ms",None)
        return output,reason
