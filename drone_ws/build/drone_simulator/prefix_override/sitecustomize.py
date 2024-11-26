import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/antreas/MsC/Q2/RO47005/finalAssignment/RO47005-Project/ro47005_drone_simulator/install/drone_simulator'
