import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/autonomous/tp_ws/src/TheElite_trajectory_planner/install/tp_package'
