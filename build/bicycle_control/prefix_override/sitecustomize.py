import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mizo/Engineering/ARL/Control_Task/Control_Project/install/bicycle_control'
