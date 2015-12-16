# from ctypes import *
from ctypes import Structure,c_uint16,c_double,c_ubyte,c_uint32,c_int16

class UI_REF(Structure):
    _pack_ = 1
    _fields_ = [("ref",    c_double*2),
		("gains",    c_double*3),
		("color",    c_double*1),
		("flag",    c_double*1)]

class H_REF(Structure):
    _pack_ = 1
    _fields_ = [("ref",    c_double*2)]

class CV_REF(Structure):
    _pack_ = 1
    _fields_ = [("ref",    c_double*3)]


CHAN_1   = 'pt-ach-sim'
CHAN_2   = 'pt-ach-real'
CHAN_3   = 'pt-ach-cv'
CHAN_4   = 'cv-pid'
CHAN_5   = 'pid-ik'
CHAN_6   = 'ik-pt_ach'
CHAN_7   = 'user-interface'

ROBOT_CHAN_VIEW   = 'robot-vid-chan'
MOTOR_CHAN = 'robot-motors'
