import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/omar/shellsim/control/track_testing_ws/src/track_visualizer/install/track_visualizer'
