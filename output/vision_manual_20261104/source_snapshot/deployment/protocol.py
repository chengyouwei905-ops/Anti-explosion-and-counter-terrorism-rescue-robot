"""Simple K230-only business messages over the existing AA55/CRC8 transport.

No ACK or automatic resend. One terminal result attempt per accepted start.
All new reports carry position only, in the commanded gripper TCP frame, mm.
The decoder retains old optional fields for archived packet inspection only.
"""
try:
    import ustruct as struct
except ImportError:
    import struct
from geometry import finite

QR_START = 0x10
BOMB_SCAN = 0x20
BOMB_MEASURE = 0x21
AIM_SCAN = 0x30
AIM_MEASURE = 0x31
RESCUE_SCAN = 0x40
RESCUE_MEASURE = 0x41
STOP = 0x7F
RESULT = 0x81
HEARTBEAT = 0x02
STATUS = 0x02  # K230 -> MCU: framed TOTALSTART!/TOTALSTOP! payloads.
START_COMMANDS = (QR_START, BOMB_SCAN, BOMB_MEASURE, AIM_SCAN, AIM_MEASURE, RESCUE_SCAN, RESCUE_MEASURE)
SCAN_COMMANDS = (BOMB_SCAN, AIM_SCAN, RESCUE_SCAN)
MEASURE_COMMANDS = (QR_START, BOMB_MEASURE, AIM_MEASURE, RESCUE_MEASURE)
NAMES = {QR_START:"QR",BOMB_SCAN:"BOMB_SCAN",BOMB_MEASURE:"BOMB_MEASURE",
         AIM_SCAN:"AIM_SCAN",AIM_MEASURE:"AIM_MEASURE",RESCUE_SCAN:"RESCUE_SCAN",
         RESCUE_MEASURE:"RESCUE_MEASURE",STOP:"STOP"}

SUCCESS, FAILED, CANCELLED, OBSERVATION_ONLY = 0, 1, 2, 3
POSITION, CONTROL_READY, QUATERNION, NORMAL = 1, 2, 4, 8
GRASP, TASK_DIGITS, SLOT_MAPPING, CANDIDATE_CALIBRATION = 16, 32, 64, 128
# TASK_DIGITS / SLOT_MAPPING remain for decoding archived packets only.
# Current output keeps task selection and the full scan mapping on K230.
VERSION = 1
HEADER_FORMAT = "<BBBBBBBBHIB"
HEADER_SIZE = 15
REASONS = {"OK":0,"CANCELLED":1,"PREEMPTED":2,"TIMEOUT":3,"UNKNOWN_COMMAND":4,
           "BAD_COMMAND_PAYLOAD":5,"TARGET_NOT_SELECTED":6,"TARGET_SLOT_NOT_KNOWN":7,
           "SLOT_MAPPING_CONFLICT":8,"MODEL_NOT_READY":9,"MODEL_CONFIGURATION_ERROR":10,
           "CALIBRATION_NOT_READY":11,"HANDEYE_CANDIDATE_ONLY":12,"SCAN_NOT_CONFIGURED":13,
           "SCAN_AMBIGUOUS":14,"CAMERA_ERROR":15,"COMMAND_QUEUE_OVERFLOW":16,
           "LINK_ERROR":17,"TABLE_CONFIGURATION_ERROR":18,"VISION_ERROR":19}


def reason_code(reason):
    if reason in REASONS: return REASONS[reason]
    if reason.startswith("MODEL_") or reason.startswith("TEST_BACKEND"): return REASONS["MODEL_CONFIGURATION_ERROR"]
    if reason.startswith("HANDEYE_") or reason.startswith("CALIBRATION_"): return REASONS["CALIBRATION_NOT_READY"]
    if reason.startswith("TABLE_") or reason.startswith("SLOT_"): return REASONS["TABLE_CONFIGURATION_ERROR"]
    if reason.startswith("SCAN_"): return REASONS["SCAN_AMBIGUOUS"]
    if reason.startswith("CAMERA_"): return REASONS["CAMERA_ERROR"]
    return REASONS["VISION_ERROR"]


def decode_command(frame):
    cmd, payload = frame.cmd, frame.payload
    if cmd == HEARTBEAT: return None
    if cmd not in START_COMMANDS+(STOP,): raise ValueError("UNKNOWN_COMMAND")
    allowed = (0,) if cmd in (QR_START,STOP) else ((0,1) if cmd in SCAN_COMMANDS else (0,1,2))
    if len(payload) not in allowed: raise ValueError("BAD_COMMAND_PAYLOAD")
    target = payload[0] if payload else 0
    slot = payload[1] if len(payload)==2 else 0
    if not 0 <= target <= 3 or not 0 <= slot <= 3: raise ValueError("BAD_COMMAND_PAYLOAD")
    return {"cmd":cmd,"input_seq":frame.seq,"target_id":target or None,"slot_id":slot or None,
            "key":(frame.seq,cmd,bytes(payload))}


def _pack_vector(values, size):
    if len(values)!=size or not all(finite(v) for v in values): raise ValueError("NONFINITE_RESULT")
    return struct.pack("<"+"f"*size,*values)


def encode_result(request, run_id, status, reason, observation=None):
    obs = observation or {}
    flags = 0
    position = obs.get("position_tool_mm")
    carries_data = status in (SUCCESS, OBSERVATION_ONLY)
    if carries_data and position is not None:
        flags |= POSITION
        if obs.get("control_ready") is True: flags |= CONTROL_READY
        if obs.get("handeye_accepted") is False: flags |= CANDIDATE_CALIBRATION
        if obs.get("grasp_position_tool_mm") is not None: flags |= GRASP
    # The MCU only needs the chosen slot and the measured pose. The task's
    # color/shape identity stays in factory.selection and factory.maps.
    target = 0
    slot = obs.get("target_slot",obs.get("slot_id",request.get("slot_id"))) or 0
    count = obs.get("sample_count",0)
    if any(type(v) is not int or not 0 <= v <= 3 for v in (target,slot)): raise ValueError("INVALID_RESULT_ID")
    if type(count) is not int or not 0 <= count <= 255: raise ValueError("INVALID_RESULT_SAMPLE_COUNT")
    frame_id = obs.get("frame_id",0) or 0
    header = struct.pack(HEADER_FORMAT,VERSION,request["cmd"],request["input_seq"],status,
                         flags,target,slot,count,run_id & 0xffff,frame_id & 0xffffffff,reason_code(reason))
    body = b""
    if flags & POSITION:
        body += _pack_vector(position,3)
        if flags & GRASP: body += _pack_vector(obs["grasp_position_tool_mm"],3)
        body += _pack_vector([obs.get("scatter_mm",0.0)],1)
    if len(header+body)>64: raise ValueError("RESULT_PAYLOAD_TOO_LONG")
    return header+body


def decode_result(payload):
    """Host/debug decoder using the exact documented byte layout."""
    if len(payload)<HEADER_SIZE: raise ValueError("SHORT_RESULT")
    values=struct.unpack(HEADER_FORMAT,payload[:HEADER_SIZE])
    version,cmd,seq,status,flags,target,slot,count,run_id,frame_id,reason=values
    if version!=VERSION: raise ValueError("RESULT_VERSION_MISMATCH")
    result={"task_cmd":cmd,"input_seq":seq,"status":status,"flags":flags,"target_id":target,
            "slot_id":slot,"sample_count":count,"run_id":run_id,"frame_id":frame_id,"reason_code":reason}
    offset=HEADER_SIZE
    def read_floats(size):
        nonlocal offset
        end=offset+size*4
        if end>len(payload): raise ValueError("SHORT_RESULT")
        data=struct.unpack("<"+"f"*size,payload[offset:end]); offset=end
        return list(data)
    if flags & POSITION:
        result["position_tool_mm"]=read_floats(3)
        if flags & QUATERNION: result["quaternion_tool_from_qr"]=read_floats(4)
        if flags & NORMAL: result["normal_tool"]=read_floats(3)
        if flags & GRASP: result["grasp_position_tool_mm"]=read_floats(3)
        if flags & TASK_DIGITS:
            if offset+3>len(payload): raise ValueError("SHORT_RESULT")
            result["task_digits"]=list(payload[offset:offset+3]); offset+=3
        result["scatter_mm"]=read_floats(1)[0]
    if flags & SLOT_MAPPING:
        if offset+3>len(payload): raise ValueError("SHORT_RESULT")
        result["slot_mapping"]=list(payload[offset:offset+3]); offset+=3
    if offset!=len(payload): raise ValueError("RESULT_LENGTH_MISMATCH")
    return result
