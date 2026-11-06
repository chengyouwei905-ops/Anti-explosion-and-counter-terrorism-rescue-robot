"""Field reference calculations; no camera, UART, or arm initialization.

B: right/forward/up. Arm command +X is left. E is calibrated gripper TCP.
These are CURRENT MCU work_reference settings, not surveyed field truth.
Pose values below are FK of current app_roboarm.c observation presets.
"""
import math

PROFILES={
    "qr":{"label":"二维码中心","source_cmd":0x10,"reference_point":"qr_center",
          "body_reference":(0.,1250.,55.),
          "preset_pose":(0.,204.662675872066,99.824932482697,-.217817090649)},
    "bomb":{"label":"爆炸物球心","source_cmd":0x21,"reference_point":"sphere_center",
            "body_reference":(-225.,0.,0.),
            "preset_pose":(154.215426385177,-4.405580176218,-4.916010291400,-.950855376487)},
    "aim":{"label":"靶心","source_cmd":0x31,"reference_point":"target_center",
           "body_reference":(-620.,0.,20.),
           "preset_pose":(250.683196776456,-7.161442587922,14.658957083631,-.234572251468)},
    "rescue":{"label":"人质轴线中点","source_cmd":0x41,"reference_point":"rescue_object_axis_midpoint",
              "body_reference":(-225.,0.,55.),
              "preset_pose":(186.048897441216,-5.314989256179,28.793387844528,-.908967474439)}
}


def _vector(values,length):
    if len(values)!=length or any(v!=v or abs(v)>=1e30 for v in values):
        raise ValueError("INVALID_REFERENCE_VECTOR")
    return list(values)


def body_from_tcp(arm_pose):
    """Mirror MCU alg_vision_transform_from_arm_pose; mm and radians."""
    x,y,z,pitch=_vector(arm_pose,4)
    if math.sqrt(x*x+y*y)<1e-6 or abs(pitch)>math.pi:raise ValueError("INVALID_REFERENCE_POSE")
    yaw=math.atan2(x,y)
    c,s,cp,sp=math.cos(yaw),math.sin(yaw),math.cos(pitch),math.sin(pitch)
    return [[c,-s*cp,s*sp],[s,c*cp,-c*sp],[0.,sp,cp]],[-x,y,z]


def tcp_to_body(point_tcp,arm_pose):
    point=_vector(point_tcp,3)
    rotation,translation=body_from_tcp(arm_pose)
    return [translation[i]+sum(rotation[i][j]*point[j] for j in range(3)) for i in range(3)]


def body_to_tcp(point_body,arm_pose):
    point=_vector(point_body,3)
    rotation,translation=body_from_tcp(arm_pose)
    return [sum(rotation[j][i]*(point[j]-translation[j]) for j in range(3)) for i in range(3)]


def expected_tcp(kind,arm_pose=None,body_reference=None):
    profile=PROFILES[kind]
    return body_to_tcp(profile["body_reference"] if body_reference is None else body_reference,
                       profile["preset_pose"] if arm_pose is None else arm_pose)


def expected_camera(kind,calibration,arm_pose=None,body_reference=None):
    """Predicted C coordinates for a configured target; not a measurement."""
    point=expected_tcp(kind,arm_pose,body_reference)
    transform=calibration.T_tool_from_camera
    if transform is None:raise ValueError("HANDEYE_NOT_CONFIGURED")
    return [sum(transform[j][i]*(point[j]-transform[j][3]) for j in range(3)) for i in range(3)]


def print_reference(kind,arm_pose=None,body_reference=None,calibration=None):
    profile=PROFILES[kind]
    pose=profile["preset_pose"] if arm_pose is None else arm_pose
    reference=profile["body_reference"] if body_reference is None else body_reference
    print("FIELD_REFERENCE",kind,"configured reference, NOT measured ground truth")
    print("ARM_POSE_FOR_REFERENCE",tuple(pose),"mm/rad; must match MCU vision_arm_pose")
    print("EXPECTED_B_MM",tuple(reference))
    print("EXPECTED_ARM_ORIGIN_RELATIVE_TARGET_B_MM",tuple(-v for v in reference))
    print("EXPECTED_TCP_MM",tuple(expected_tcp(kind,pose,reference)))
    if calibration is not None:
        camera=expected_camera(kind,calibration,pose,reference)
        print("PREDICTED_CAMERA_MM",tuple(camera))
        if camera[2]<=0:
            print("REFERENCE_VIEW_WARNING: configured target is behind the camera")
        else:
            width,height=calibration.camera.image_size
            ideal=(calibration.camera.fx*camera[0]/camera[2]+calibration.camera.cx,
                   calibration.camera.fy*camera[1]/camera[2]+calibration.camera.cy)
            print("PREDICTED_UNDISTORTED_CENTER_VGA_PX",ideal)
            if not (0<=ideal[0]<width and 0<=ideal[1]<height):
                # Brown distortion polynomials can fold at extreme rays. A
                # seemingly in-image extrapolated pixel is not evidence of view.
                print("REFERENCE_VIEW_WARNING: outside nominal calibrated view; distortion extrapolation is unreliable")
            else:
                pixel=calibration.camera.project_ray(camera)
                print("PREDICTED_CENTER_VGA_PX",tuple(pixel))
            if kind in ("aim","rescue") and camera[2]<100:
                print("REFERENCE_VIEW_WARNING: below current 100-mm minimum measurement depth")
    return expected_tcp(kind,pose,reference)
