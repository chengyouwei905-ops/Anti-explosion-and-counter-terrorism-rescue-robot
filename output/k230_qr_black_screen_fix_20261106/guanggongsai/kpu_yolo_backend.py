"""Native K230 KPU: exact packed-RGB letterbox + bounded YOLOv8 decoding."""
import math,gc
from geometry import finite


def letterbox_transform(raw_size,input_size,center=True):
    w,h=raw_size;ow,oh=input_size
    scale=min(ow/float(w),oh/float(h))
    rw,rh=int(round(w*scale)),int(round(h*scale))
    left=int(round((ow-rw)/2.0-0.1)) if center else 0
    top=int(round((oh-rh)/2.0-0.1)) if center else 0
    return {"raw_size":list(raw_size),"scale_x":rw/float(w),"scale_y":rh/float(h),
            "pad_x":left,"pad_y":top,"padding":[top,oh-rh-top,left,ow-rw-left]}


def _iou(a,b):
    x=max(a[0],b[0]);y=max(a[1],b[1])
    w=max(0.0,min(a[0]+a[2],b[0]+b[2])-x)
    h=max(0.0,min(a[1]+a[3],b[1]+b[3])-y)
    inter=w*h
    return inter/max(a[2]*a[3]+b[2]*b[3]-inter,1e-9)


def _prob(value):
    value=float(value)
    if not finite(value):raise ValueError("MODEL_NONFINITE_OUTPUT")
    # Small quantization overshoots are permitted; logits require another decoder.
    if value < -0.08 or value > 1.08:raise ValueError("MODEL_PROBABILITY_CONTRACT_MISMATCH")
    return min(1.0,max(0.0,float(value)))


def decode_outputs(outputs,metadata):
    contract=metadata["output_contract"]
    threshold=float(metadata.get("confidence_threshold",0.55))
    limit=int(metadata.get("max_candidates",96));candidates=[]
    classes=len(metadata["labels"])
    def add(cls,score,cx,cy,w,h):
        if score<threshold:return
        values=(cx,cy,w,h)
        if not all(finite(v) for v in values) or min(w,h)<=0:raise ValueError("MODEL_INVALID_BOX")
        item={"class_id":int(cls),"score":float(score),"bbox_xywh":[float(cx-w/2),float(cy-h/2),float(w),float(h)]}
        candidates.append(item)
        if len(candidates)>limit:
            candidates.sort(key=lambda r:r["score"],reverse=True)
            del candidates[limit:]
    if contract["kind"]=="yolov8_xywh_class_probabilities":
        if len(outputs)!=1 or tuple(outputs[0].shape)!=(1,4+classes,contract["boxes"]):
            raise ValueError("MODEL_OUTPUT_SHAPE_MISMATCH")
        array=outputs[0][0]
        for i in range(contract["boxes"]):
            scores=[_prob(array[4+c,i]) for c in range(classes)]
            cls=max(range(classes),key=lambda c:scores[c])
            if scores[cls]>=threshold:add(cls,scores[cls],*[float(array[c,i]) for c in range(4)])
    else:raise ValueError("MODEL_UNKNOWN_OUTPUT_CONTRACT")
    candidates.sort(key=lambda r:r["score"],reverse=True)
    selected=[]
    for item in candidates:
        if any(item["class_id"]==old["class_id"] and _iou(item["bbox_xywh"],old["bbox_xywh"])>metadata.get("nms_iou",0.45) for old in selected):continue
        selected.append(item)
        if len(selected)>=metadata.get("max_detections",12):break
    return selected


class KPUYoloBackend:
    is_test_backend=False
    def __init__(self):
        self.kpu=self.ai2d=self.builder=self.output_tensor=self.output_array=None
        self.nn=self.np=self.aidemo=None
        self.metadata=self.transform=None
    def load(self,metadata):
        self.close()
        import nncase_runtime as nn
        import ulab.numpy as np
        self.nn,self.np,self.metadata=nn,np,metadata
        try:
            import aidemo
            self.aidemo=aidemo if hasattr(aidemo,"yolov8_det_postprocess") else None
        except ImportError:self.aidemo=None
        if metadata["input_dtype"]!="uint8" or metadata["tensor_layout"]!="NCHW" or metadata["color_order"]!="RGB":
            raise ValueError("MODEL_UNSUPPORTED_INPUT_CONTRACT")
        self.kpu=nn.kpu()
        try:
            self.kpu.load_kmodel(metadata["model_path"])
            w,h=metadata["input_size"]
            desc=self.kpu.inputs_desc(0)
            if self.kpu.inputs_size()!=1 or desc["tensor_datatype"]!="dt_uint8" or desc["tensor_size"]!=3*w*h:
                raise ValueError("MODEL_INPUT_DESCRIPTOR_MISMATCH")
            expected=metadata["output_contract"]["output_bytes"]
            if self.kpu.outputs_size()!=len(expected):raise ValueError("MODEL_OUTPUT_DESCRIPTOR_MISMATCH")
            for i,size in enumerate(expected):
                desc=self.kpu.outputs_desc(i)
                if desc["tensor_datatype"]!="dt_float32" or desc["tensor_size"]!=size:raise ValueError("MODEL_OUTPUT_DESCRIPTOR_MISMATCH")
            self.transform=letterbox_transform(metadata["raw_size"],metadata["input_size"],metadata["preprocess"].get("center",True))
            self.output_array=np.zeros((1,3,h,w),dtype=np.uint8)
            self.output_tensor=nn.from_numpy(self.output_array)
            self.kpu.set_input_tensor(0,self.output_tensor)
            self.ai2d=nn.ai2d()
            self.ai2d.set_dtype(nn.ai2d_format.RGB_packed,nn.ai2d_format.NCHW_FMT,np.uint8,np.uint8)
            padding=self.transform["padding"]
            fill=metadata["preprocess"]["fill"]
            self.ai2d.set_pad_param(True,[0,0,0,0]+padding,0,[fill,fill,fill])
            self.ai2d.set_resize_param(True,nn.interp_method.tf_bilinear,nn.interp_mode.half_pixel)
            raw_w,raw_h=metadata["raw_size"]
            self.builder=self.ai2d.build([1,raw_h,raw_w,3],[1,3,h,w])
            return {"ok":True}
        except BaseException:
            self.close();raise
    def infer(self,raw):
        if self.kpu is None:raise ValueError("MODEL_NOT_LOADED")
        w,h=self.metadata["raw_size"]
        array=raw.to_numpy_ref()
        if tuple(array.shape)!=(h,w,3):raise ValueError("MODEL_EXPECTS_PACKED_RGB888_RAW")
        source=self.nn.from_numpy(array.reshape((1,h,w,3)))
        outputs=[]
        try:
            self.builder.run(source,self.output_tensor)
            self.kpu.run()
            for i in range(self.kpu.outputs_size()):outputs.append(self.kpu.get_output_tensor(i).to_numpy())
            if self.aidemo is not None:
                # Equal source/model/display sizes make this C postprocessor an
                # identity coordinate mapping. Our exact letterbox inverse is
                # still the single owner of raw-image coordinate restoration.
                h,w=self.metadata["input_size"][1],self.metadata["input_size"][0]
                array=outputs[0]
                expected=(1,4+len(self.metadata["labels"]),self.metadata["output_contract"]["boxes"])
                if tuple(array.shape)!=expected:raise ValueError("MODEL_OUTPUT_SHAPE_MISMATCH")
                packed=array[0].transpose().copy()
                # Separate C calls preserve class-aware NMS even on firmware
                # builds whose combined-class helper uses global suppression.
                scratch=self.np.zeros((expected[2],5),dtype=array.dtype)
                detections=[]
                for cls in range(len(self.metadata["labels"])):
                    scratch[:,:4]=packed[:,:4]
                    scratch[:,4]=packed[:,4+cls]
                    result=self.aidemo.yolov8_det_postprocess(scratch,[h,w],[h,w],[h,w],1,
                        self.metadata.get("confidence_threshold",0.55),self.metadata.get("nms_iou",0.45),self.metadata.get("max_detections",12))
                    if result:
                        for box,score in zip(result[0],result[2]):
                            detections.append({"class_id":cls,"score":_prob(score),"bbox_xywh":[float(v) for v in box]})
                detections.sort(key=lambda item:item["score"],reverse=True)
                detections=detections[:self.metadata.get("max_detections",12)]
                scratch=packed=result=None
            else:
                detections=decode_outputs(outputs,self.metadata)
            return {"space":"network_pixels","detections":detections,"transform":self.transform}
        finally:
            outputs=[]
            source=None
    def close(self):
        # Release references before shrinking the global runtime pool.
        for name in ("builder","ai2d","kpu"):
            resource=getattr(self,name,None)
            if resource is not None:
                try:resource.release()
                except Exception:pass
            setattr(self,name,None)
        self.output_tensor=self.output_array=None
        gc.collect()
        if self.nn is not None:
            try:self.nn.shrink_memory_pool()
            except Exception:pass
