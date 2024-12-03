import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mukil/MSc_Robotics/Quarter2/Planning_&_Decision_Making_(RO47005)/Project/RO47005-Project/drone_ws/install/scenario_creator'
