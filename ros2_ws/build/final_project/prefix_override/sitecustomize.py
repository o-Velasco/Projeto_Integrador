import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/francisco/crazyflie_mapping_demo/ros2_ws/install/final_project'
