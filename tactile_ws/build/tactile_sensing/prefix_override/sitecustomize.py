import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/amey/robot_learning/tactile_ws/install/tactile_sensing'
