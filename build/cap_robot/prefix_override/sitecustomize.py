import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kimhakjin/colcon_ws/src/cap_robot/install/cap_robot'
