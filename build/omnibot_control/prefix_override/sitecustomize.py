import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/chongzhiw/Gazebo_Repo/ROS2_omnibot/install/omnibot_control'
