'''
Created on 2019. 3. 29.

@author: JSK
''' 

DICT_ERROR = { \
    'STATE_NORMAL': 0,
    'UNKNOWN_ERROR': 1,
    'CAMERA': 10000,
    'CAMERA_INIT': 10001,
    'CAMERA_GET_FRAME': 10002,
    'VISIONPROC': 20000,
    'VISIONPROC_INIT': 20001,
    'VISIONPROC_INTERPRETATION': 20002,
    'VISIONPROC_DETECTOR': 21000,
    'VISIONPROC_DETECTOR_NO_ROBOT': 21001,
    'VISIONPROC_DETECTOR_INVALID_CLASS': 21002,
    'VISIONPROC_DETECTOR_ALIGN_Z': 21003,
    'VISIONPROC_DETECTOR_FIT_FLOOR': 21004,
    'VISIONPROC_DETECTOR_NO_VALID_DETECTION': 21005,
    'VISIONPROC_DETECTOR_NO_DETECTION_TO_RETRIVE': 21006,
    'VISIONPROC_DETECTOR_GRIP_SCAN_FAIL': 21007,
    'VISIONPROC_DETECTOR_THREAD': 21009,
    'VISIONPROC_DETECTOR_NOIMPLEMENTATION_SEG_SETPARAM': 21011,
    'VISIONPROC_DETECTOR_NOIMPLEMENTATION_SEG_RUN': 21012,
    'VISIONPROC_DETECTOR_NOIMPLEMENTATION_POS_SETPARAM': 21021,
    'VISIONPROC_DETECTOR_NOIMPLEMENTATION_POS_RUN': 21022,
    'VISIONPROC_DETECTOR_NOIMPLEMENTATION_PST_SETPARAM': 21031,
    'VISIONPROC_DETECTOR_NOIMPLEMENTATION_PST_RUN': 21032,
    'VISIONPROC_DETECTOR_MRCNN': 21100,
    'VISIONPROC_DETECTOR_MRCNN_INFERENCE': 21101,
    'VISIONPROC_DETECTOR_MRCNN_GET_TCO': 21102,
    'VISIONPROC_DETECTOR_MRCNN_NO_VALID_DETECTION': 21103,
    'VISIONPROC_DETECTOR_GRIP_FAIL': 21090,
    'VISIONPROC_CONTROLLER_TOOL_LOAD_FAIL': 22000,
    'VISIONPROC_CONTROLLER_GRIP_LOAD_FAIL': 22001,
    'VISIONPROC_CONTROLLER_JOINT_REUEST_NO_ROBOT': 22011,
    'CALIBPROC_CALIBRATOR_NOT_IMPLEMENTED_MODE': 31001,
    'TRAIN_SERVER_NOIMPLEMENTATION_SETPARAM': 51011,
    'TRAIN_SERVER_NOIMPLEMENTATION_TRAIN': 51012,
    'TRAIN_SERVER_NOIMPLEMENTATION_CHECKPOINT': 51013,
    'TRAIN_SERVER_GET_CHECKPOINT_INPROGRESS': 51021
    }
ERROR_NAMES = DICT_ERROR.keys()
ERROR_CODES = DICT_ERROR.values()

def CODE2ERROR(CODE, dict_error = DICT_ERROR):
    for k,v in dict_error.items():
        if v == CODE:
            return k
    return 'UNKNOWN_ERROR'

def ERROR2CODE(ERROR, dict_error = DICT_ERROR):
    if ERROR in DICT_ERROR.keys():
        return DICT_ERROR[ERROR]
    else:
        return DICT_ERROR['UNKNOWN_ERROR']


GLOBAL_ERROR_STATE = DICT_ERROR['STATE_NORMAL']

class ERROR_DESCRIPTER(BaseException):
    def __init__(self, error='UNKNOWN_ERROR'):
        self.error = error
        self.code = ERROR2CODE(error)
    def __str__(self):
        return self.error