import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/addinnedu/monitoring_camera_ws/install/aruco_marker_pkg'
