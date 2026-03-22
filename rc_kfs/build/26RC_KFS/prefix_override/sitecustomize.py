import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ray/ros2_ws/src/rc_kfs/install/rc_kfs'
