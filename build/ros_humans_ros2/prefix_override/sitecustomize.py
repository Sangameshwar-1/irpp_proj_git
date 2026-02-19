import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sangam/Documents/Acad/sem-4/IRPP/PROJ/install/ros_humans_ros2'
