import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nb_adm/SWARMz4/ros2_ws/install/px4_offboard_velocity'
