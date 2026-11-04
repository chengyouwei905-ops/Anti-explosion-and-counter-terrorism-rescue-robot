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
           "MULTIPLE_QRS_AMBIGUOUS":"画面中有多个二维码",
           "INVALID_TASK_CODE":"任务码须为三个1至3数字",
           "HANDEYE_CANDIDATE_ONLY":"手眼未确认，仅供观察",
           "CANCELLED":"MCU取消了当前任务","PREEMPTED":"新任务中断扫码",
           "QR_SAMPLE_EXPIRED":"二维码样本已过期",
           "STALE_OR_DUPLICATE_FRAME":"重复图像未计入采样",
           "calibrated_raw_geometry_not_confirmed":"二维码图像几何未确认",
           "corner_orientation_unknown":"二维码角点方向未知",
           "corner_orientation_unverified":"二维码角点顺序未验证",
           "qr_too_small_or_duplicate_corners":"二维码太小或角点重合",
           "degenerate_or_edge_on_quadrilateral":"二维码过于倾斜",
           "nonconvex_mirrored_or_wrong_corner_winding":"二维码角点顺序异常",
           "no_converged_position_model":"二维码定位求解未收敛",
           "reprojection_error_too_large":"二维码重投影误差过大",
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
    if reason.startswith("QR_DETECTION_ERROR:"):return "二维码检测调用异常"
    if reason.startswith("qr_interface_or_adapter_error:"):return "二维码角点接口异常"
    if reason.startswith("image_size_mismatch"):return "二维码图像尺寸不匹配"
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


def qr_debug_panel(manager,state,reason,count):
    """Build LCD-only QR diagnostics. Never change sampling or result packets."""
    import protocol as p
    import runtime_config as cfg
    from config import QR_MAX_CENTER_DISAGREEMENT_MM,QR_MAX_POSITION_3SIGMA_MM
    q=getattr(manager,"qr_debug",None) or {}
    obs=q.get("observation") or {}
    active=manager.request is not None
    last=manager.last_report or {}
    terminal=q.get("terminal_reason") if not active else None
    reason=terminal or reason
    cause=q.get("last_error")
    samples=count if active else q.get("samples",count)
    color=(255,190,60)
    state_text={"SETTLING":"等待停稳","RUNNING":"识别中","REPORT":"准备回传",
                "PAUSED":"已暂停","IDLE":"任务结束"}.get(state,state)
    if state=="PAUSED":
        title="二维码任务已暂停"
    elif reason in ("CANCELLED","PREEMPTED"):
        title="二维码已取消" if reason=="CANCELLED" else "二维码被新任务中断"
        color=(255,100,90)
    elif reason=="TIMEOUT":
        title="二维码识别超时"
        color=(255,100,90)
    elif terminal is not None and last.get("status")==p.SUCCESS:
        title="二维码定位成功"
        color=(70,255,130)
    elif terminal is not None and last.get("status")==p.OBSERVATION_ONLY:
        title="二维码仅供观察"
    elif reason in ("OK","MEASUREMENT_STABILIZING"):
        title="二维码有效定位 %d/%d" % (samples,cfg.MEASUREMENT_SAMPLES)
    elif reason in ("STARTING","SETTLING","IDLE"):
        title="二维码等待停稳" if state=="SETTLING" else "二维码等待首帧"
    else:
        title="二维码："+status_text(reason)
        color=(255,100,90)
    elapsed=q.get("elapsed_ms",0)
    if active:
        clock=getattr(manager,"_elapsed",None)
        if clock is not None:elapsed=clock()
    def metric(key):
        value=obs.get(key)
        return "%.1f" % value if value is not None and finite(value) else "--"
    center,sigma=metric("center_disagreement_mm"),metric("position_3sigma_mm")
    code=str(reason)
    if terminal and cause and cause!=reason:code+=" <- "+cause
    detail="采样:%d/%d 耗时:%.1fs 中心差:%s/%.0fmm 3sigma:%s/%.0fmm" % (
        samples,cfg.MEASUREMENT_SAMPLES,elapsed/1000.,center,QR_MAX_CENTER_DISAGREEMENT_MM,
        sigma,QR_MAX_POSITION_3SIGMA_MM)
    payload=str(obs.get("payload") or "--").replace("\n"," ").replace("\r"," ")[:12]
    lines=["二维码调试",state_text,
           "连续:%d/%d" % (samples,cfg.MEASUREMENT_SAMPLES) if cfg.QR_SAMPLE_WINDOW_MS==0 else
           "窗口:%d/%d" % (samples,cfg.MEASUREMENT_SAMPLES),
           "耗时:%.1f/%.0fs" % (elapsed/1000.,cfg.MEASUREMENT_TIMEOUT_MS/1000.),
           "检测:%s 个" % obs.get("detected_count","--"),"载荷:"+payload,
           "原因:",status_text(reason),"代码:",str(reason),
           "中心差:%s/%.0fmm" % (center,QR_MAX_CENTER_DISAGREEMENT_MM),
           "3sigma:%s/%.0fmm" % (sigma,QR_MAX_POSITION_3SIGMA_MM)]
    if cause:lines.extend(["最近失败:",status_text(cause),cause])
    if obs.get("contrast_retry_error"):lines.extend(["对比度重试失败:",obs["contrast_retry_error"]])
    mode=obs.get("detection_mode")
    if mode:lines.append("图像:"+("对比度重试" if mode=="contrast" else "原始灰度"))
    lines.append("帧:%d 最高:%d" % (q.get("frames",0),q.get("peak_samples",0)))
    return {"lines":lines,"notice":{"title":title,"code":code,"detail":detail,"color":color}}
