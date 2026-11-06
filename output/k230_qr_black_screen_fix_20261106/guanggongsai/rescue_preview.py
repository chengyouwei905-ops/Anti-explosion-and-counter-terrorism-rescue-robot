"""Manual LCD check for the rescue model. Run this separately from competition_main."""
import os,sys,time,gc
PROJECT_ROOT='/data/guanggongsai'
if PROJECT_ROOT not in sys.path:sys.path.insert(0,PROJECT_ROOT)
import runtime_config as cfg
from calibration_params import load_calibration
from task_runner import VisionTaskFactory
from camera_service import CameraService
from rescue_config import SHAPE_NAMES,SCAN_GATE_RAW_PX

def main(max_frames=None):
    os.exitpoint(os.EXITPOINT_ENABLE)
    camera=factory=frame=preview=None
    count=0
    try:
        cal=load_calibration(PROJECT_ROOT,handeye_accepted_override=cfg.HANDEYE_ACCEPTED_BY_USER)
        factory=VisionTaskFactory(cal,PROJECT_ROOT)
        detector=factory._model('rescue')
        camera=CameraService()
        while max_frames is None or count<max_frames:
            os.exitpoint()
            frame=camera.get_frame()
            if frame is None:
                if camera.state=='ERROR':raise RuntimeError(camera.error)
                time.sleep_ms(5)
                continue
            started=time.ticks_ms()
            detection=detector.detect(frame['image'])
            elapsed=time.ticks_diff(time.ticks_ms(),started)
            if not detection['ok']:raise RuntimeError(detection['reason'])
            preview=frame['image'].to_rgb565()
            labels=[]
            for item in detection['detections']:
                x,y,w,h=[int(v) for v in item['bbox_raw_px']]
                shape=item['shape_id']
                color=((255,80,80),(80,255,80),(80,160,255))[shape-1]
                preview.draw_rectangle(x,y,w,h,color=color,thickness=2)
                text='%d %s %.2f'%(shape,SHAPE_NAMES[shape],item['score'])
                preview.draw_string_advanced(max(0,x),max(0,y-20),18,text,color=color)
                labels.append(text)
            camera.render_debug(preview,{'lines':['救援模型检查','帧:%d'%frame['frame_id'],
                '推理:%d ms'%elapsed,'目标:%d'%len(labels),'1:圆柱','2:圆台','3:腰鼓']+labels,
                'scan_gate':SCAN_GATE_RAW_PX})
            if camera.display_error:raise RuntimeError(camera.display_error)
            if count%10==0:print('RESCUE_PREVIEW',frame['frame_id'],elapsed,labels)
            count+=1
            frame=preview=None
            gc.collect()
    except KeyboardInterrupt:
        print('RESCUE_PREVIEW_STOPPED')
    finally:
        frame=preview=None
        try:
            if factory is not None:factory.close()
        finally:
            if camera is not None:camera.close()
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
            time.sleep_ms(100)
            gc.collect()

if __name__=='__main__':main()
