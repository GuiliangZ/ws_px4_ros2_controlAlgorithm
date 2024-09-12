import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/guiliang-linux/Documents/projects/DroneProject/ws_px4_ros2_controlAlgorithm/install/px4_offboard'
