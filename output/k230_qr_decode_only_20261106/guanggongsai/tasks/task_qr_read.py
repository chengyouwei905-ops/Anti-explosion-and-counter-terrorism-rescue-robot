"""QR task-code decoding only. No pose estimator, hand-eye transform or averaging."""
from tasks.task_qr import parse_task


class QRReadTask:
    def __init__(self,contrast_retry=False,contrast_retry_every=3,
                 roi_size=None,roi_reference_size=None):
        if type(contrast_retry_every) is not int or contrast_retry_every<1:
            raise ValueError("INVALID_QR_CONTRAST_RETRY_INTERVAL")
        self.contrast_retry=bool(contrast_retry)
        self.contrast_retry_every=contrast_retry_every
        self.roi_size=None if roi_size is None else tuple(roi_size)
        self.roi_reference_size=None if roi_reference_size is None else tuple(roi_reference_size)
        self.last_frame=-1
        self.image_size=(640,480)
        self.mode="READ"
        self.reset_window()

    def reset_window(self):
        self.missed_detections=0
        if self.mode!="CANCELLED":self.mode="READ"

    def detection_roi(self,image_size):
        if self.roi_size is None:return None
        if len(self.roi_size)!=2 or any(type(v) is not int or v<=0 for v in self.roi_size):
            raise ValueError("INVALID_QR_ROI_SIZE")
        iw,ih=image_size
        width,height=self.roi_size
        if self.roi_reference_size is not None:
            if (len(self.roi_reference_size)!=2 or
                    any(type(v) is not int or v<=0 for v in self.roi_reference_size)):
                raise ValueError("INVALID_QR_ROI_REFERENCE_SIZE")
            rw,rh=self.roi_reference_size
            width=max(1,int(round(width*iw/float(rw))))
            height=max(1,int(round(height*ih/float(rh))))
        if width>iw or height>ih:raise ValueError("QR_ROI_OUTSIDE_IMAGE")
        return ((iw-width)//2,(ih-height)//2,width,height)

    def measure(self,raw_gray,frame_id,geometry_confirmed=False,stable=False):
        result={"ok":False,"qr_info_only":True,"decoded":False,"task_valid":False,
                "pose_valid":False,"control_ready":False,"frame_id":frame_id,"sample_count":0}
        if self.mode!="READ":
            result["reason"]="TASK_NOT_ACTIVE";return result
        if type(frame_id) is not int or frame_id<=self.last_frame:
            result["reason"]="STALE_OR_DUPLICATE_FRAME";return result
        self.last_frame=frame_id
        if not stable:
            result["reason"]="OBSERVATION_NOT_STABLE";return result
        enhanced=None
        try:
            size=(raw_gray.width(),raw_gray.height())
            self.image_size=size
            roi=self.detection_roi(size)
            result.update(detection_roi=roi,detection_image_size=size,detection_mode="raw")
            codes=raw_gray.find_qrcodes(roi=roi) if roi is not None else raw_gray.find_qrcodes()
            if codes:self.missed_detections=0
            else:
                self.missed_detections+=1
                if self.contrast_retry and self.missed_detections>=self.contrast_retry_every:
                    self.missed_detections=0
                    try:
                        enhanced=raw_gray.copy();enhanced.histeq(adaptive=False)
                        codes=enhanced.find_qrcodes(roi=roi) if roi is not None else enhanced.find_qrcodes()
                        result["detection_mode"]="contrast"
                    except Exception as exc:
                        result["contrast_retry_error"]=str(exc)
            if not codes:
                result["reason"]="QR_NOT_FOUND";return result
            result["decoded"]=True
            if len(codes)!=1:
                result["reason"]="MULTIPLE_QRS_AMBIGUOUS";return result
            payload=codes[0].payload()
            result["payload"]=payload
            try:task=parse_task(payload)
            except ValueError:
                result["reason"]="INVALID_TASK_CODE";return result
            result.update(ok=True,task_valid=True,task=task,reason="OK")
            self.mode="DONE"
            return result
        except Exception as exc:
            result["reason"]="QR_DETECTION_ERROR:"+str(exc)
            return result
        finally:
            enhanced=None

    def cancel(self):
        self.mode="CANCELLED"
        self.reset_window()
