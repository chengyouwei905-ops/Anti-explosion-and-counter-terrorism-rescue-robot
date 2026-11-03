"""Detection metadata and display-only overlays; no inference or hardware ownership."""
from geometry import finite

LABELS={"bomb":(None,"红球","绿球","蓝球"),
        "aim":(None,"红靶","绿靶","蓝靶"),
        "rescue":(None,"圆柱","圆台","腰鼓")}


def image_size(image,default=(640,480)):
    width,height=getattr(image,"width",None),getattr(image,"height",None)
    return (width(),height()) if callable(width) and callable(height) else default


def target_label(kind,target_id):
    return "二维码" if kind=="qr" else LABELS[kind][target_id]


class DetectionTrace:
    def __init__(self,kind,target_id):
        self.kind,self.target_id=kind,target_id
        self.begin(-1,(640,480))

    def begin(self,frame_id,size):
        self.frame_id,self.size=frame_id,size
        self.detections=[]
        self.error=None

    def detector_result(self,result):
        self.detections=[]
        if not result.get("ok"):
            self.error=result.get("reason","DETECTION_ERROR")
        for item in result.get("detections",[]):
            box=item.get("bbox_raw_px")
            if box is None or len(box)!=4 or not all(finite(v) for v in box):continue
            identity=item.get("shape_id") if self.kind=="rescue" else item.get("color_id")
            if type(identity) is not int or not 1<=identity<=3:continue
            selected=identity==self.target_id
            in_roi=True
            if self.kind in ("bomb","aim"):
                if self.kind=="bomb":
                    from config import BOMB_DETECTION_ROI_RAW_PX
                    x,y,w,h=BOMB_DETECTION_ROI_RAW_PX
                else:
                    from aim_config import TARGET_DETECTION_ROI_RAW_PX
                    x,y,w,h=TARGET_DETECTION_ROI_RAW_PX
                cx,cy=box[0]+box[2]*.5,box[1]+box[3]*.5
                in_roi=x<=cx<x+w and y<=cy<y+h
            self.detections.append({"bbox_raw_px":tuple(box),"label":target_label(self.kind,identity),
                                    "selected":selected,"score":item.get("score"),"in_roi":in_roi})

    def qr_result(self,results):
        self.detections=[]
        for item in results:
            corners=item.get("corners_px")
            if not corners or len(corners)!=4:continue
            if any(len(p)!=2 or not all(finite(v) for v in p) for p in corners):continue
            xs,ys=[p[0] for p in corners],[p[1] for p in corners]
            payload=str(item.get("payload") or "").replace("\n"," ")[:12]
            self.detections.append({"bbox_raw_px":(min(xs),min(ys),max(xs)-min(xs),max(ys)-min(ys)),
                                    "corners_px":tuple(tuple(p) for p in corners),
                                    "label":"二维码:"+payload,"selected":True,"score":None,"in_roi":True})

    def snapshot(self):
        return {"frame_id":self.frame_id,"source_size":self.size,
                "detections":list(self.detections),"error":self.error}


class DetectorProbe:
    def __init__(self,detector,trace):self.inner,self.trace=detector,trace
    def __getattr__(self,name):return getattr(self.inner,name)
    def detect(self,raw):
        result=self.inner.detect(raw)
        try:self.trace.detector_result(result)
        except Exception as exc:self.trace.error="DISPLAY_METADATA_ERROR:"+str(exc)
        return result


class QRPoseProbe:
    def __init__(self,estimator,trace):self.inner,self.trace=estimator,trace
    def __getattr__(self,name):return getattr(self.inner,name)
    def detect(self,gray,*args,**kwargs):
        result=self.inner.detect(gray,*args,**kwargs)
        try:
            self.trace.size=image_size(gray,self.inner.image_size)
            self.trace.qr_result(result)
        except Exception as exc:self.trace.error="DISPLAY_METADATA_ERROR:"+str(exc)
        return result


def status_text(reason):
    if not isinstance(reason,str):reason=str(reason)
    texts={"PAUSED":"已暂停","SETTLING":"等待停稳","WAIT_NEXT_BATCH":"检测中/待采样",
           "SENT":"已发送坐标","OBSERVATION_ONLY":"已发观察结果",
           "MEASUREMENT_STABILIZING":"采样中","POSITION_NOT_STABLE":"位置波动",
"QR_NOT_FOUND":"未找到二维码",
           "TARGET_BOMB_NOT_FOUND":"未找到目标球","TARGET_NOT_FOUND":"未找到目标靶",
           "RESCUE_TARGET_NOT_FOUND":"未找到目标人质",
           "BOMB_TARGET_OUTSIDE_DETECTION_ROI":"目标在矩形外",
           "TARGET_OUTSIDE_DETECTION_ROI":"靶心在矩形外",
           "CONTOUR_CLIPPED":"轮廓不完整","DETECTION_CLIPPED":"检测框被截断",
           "TARGET_CONTOUR_CLIPPED":"靶轮廓不完整","RESCUE_TARGET_CLIPPED":"人质轮廓不完整",
           "SPHERE_CONTOUR_INCONSISTENT":"球形轮廓拟合不一致",
           "SPHERE_CONTOUR_INCOMPLETE":"球面轮廓覆盖不足",
           "CONTOUR_POINT_COUNT":"可用轮廓点不足",
           "TARGET_RINGS_TOO_SMALL":"可用色环边缘不足",
           "COLOR_COMPONENT_TOO_SMALL":"颜色区域太小","MULTIPLE_COLOR_COMPONENTS":"颜色区域歧义",
           "SPHERE_RANGE_INVALID":"球心距离超出范围","NONPOSITIVE_DEPTH":"球心深度无效",
           "CONTOUR_TOO_COMPLEX":"颜色轮廓过于复杂","NONFINITE_MEASUREMENT":"坐标计算无效",
           "OBSERVATION_NOT_STABLE":"尚未停稳","CALIBRATION_NOT_READY":"标定尚未就绪",
           "VISION_ERROR":"定位计算异常，详见终端",
           "MEASUREMENT_AFTER_TIMEOUT":"定位计算完成时已超时",
           "TARGET_CENTER_AMBIGUOUS":"靶心位置存在歧义",
           "TARGET_POSITION_POORLY_OBSERVABLE":"靶心位置不够可靠",
           "TARGET_GEOMETRY_UNOBSERVABLE":"靶环无法确定位置",
           "TWO_COMPLETE_RINGS_REQUIRED":"内外环轮廓点不足",
           "qr_center_ambiguous":"二维码中心位置歧义","qr_position_uncertain":"二维码位置不可靠"}
    texts.update({"STARTING":"任务初始化","IDLE":"等待MCU指令","OK":"任务完成","TIMEOUT":"识别超时",
                  "SCAN_WAIT_TARGET":"扫描中，等待目标","SCAN_STABILIZING":"正在确认当前目标",
                  "SCAN_TARGET_RECORDED":"已记录一个位置","SCAN_ALREADY_COUNTED":"该目标已计数",
                  "SCAN_WAIT_SINGLE_TARGET":"多个目标入框，等待单个目标",
                  "SCAN_MULTIPLE_TARGETS_IN_GATE":"多个目标入框，顺序不明确",
                  "SCAN_ORDER_AMBIGUOUS_RESTART_REQUIRED":"扫描已失败，需重新开始",
                  "SCAN_TRANSITION_WITHOUT_RELEASE":"目标切换缺少间隔",
                  "SCAN_REVISIT_ORDER_AMBIGUOUS":"重复经过目标，顺序不明确"})
    if reason in texts:return texts[reason]
    if "SHAPE_MISMATCH" in reason:return "图像或模型尺寸不匹配"
    if reason.startswith("DETECTION_ERROR:") or reason.startswith("TARGET_DETECTOR_ERROR:"):return "检测异常，详见错误代码"
    if reason.startswith("MULTIPLE_"):return "存在多个同类目标"
    if reason.startswith("CAMERA_"):return "相机准备中"
    if reason.startswith("MODEL_"):return "模型异常"
    return "定位未通过"


def draw_detections(canvas,detections,source_size,pose_valid=None,preview_size=(640,480)):
    sw,sh=source_size
    width,height=preview_size
    if sw<=0 or sh<=0:return
    sx,sy=width/float(sw),height/float(sh)
    for item in detections:
        x,y,w,h=item["bbox_raw_px"]
        if w<=0 or h<=0:continue
        x0,y0=max(0,int(x*sx)),max(0,int(y*sy))
        x1,y1=min(width-1,int((x+w)*sx)),min(height-1,int((y+h)*sy))
        if x1<=x0 or y1<=y0:continue
        selected=item.get("selected",False)
        color=((255,80,80) if not item.get("in_roi",True) else
               ((0,255,100) if pose_valid is True else (255,165,0))) if selected else (150,180,210)
        canvas.draw_rectangle(x0,y0,x1-x0,y1-y0,color=color,thickness=3 if selected else 1)
        cx,cy=(x0+x1)//2,(y0+y1)//2
        canvas.draw_line(max(0,cx-5),cy,min(width-1,cx+5),cy,color=color,thickness=2)
        canvas.draw_line(cx,max(0,cy-5),cx,min(height-1,cy+5),color=color,thickness=2)
        corners=item.get("corners_px")
        if corners:
            points=[(max(0,min(width-1,int(p[0]*sx))),max(0,min(height-1,int(p[1]*sy)))) for p in corners]
            for a,b in zip(points,points[1:]+points[:1]):
                canvas.draw_line(a[0],a[1],b[0],b[1],color=color,thickness=2)
        label=item["label"]
        score=item.get("score")
        if score is not None and finite(score):label+=" %d%%" % int(score*100+.5)
        if selected:label+=" 区外" if not item.get("in_roi",True) else (" 有效" if pose_valid is True else " 已检测")
        text_width=sum(14 if ord(c)>127 else 7 for c in label)
        tx=max(0,min(x0,width-text_width))
        ty=max(0,y0-18)
        canvas.draw_string_advanced(tx,ty,14,label,color=color)
