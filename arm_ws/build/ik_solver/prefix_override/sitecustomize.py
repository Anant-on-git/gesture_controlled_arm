import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/anant/gesture_controlled_arm/arm_ws/install/ik_solver'
