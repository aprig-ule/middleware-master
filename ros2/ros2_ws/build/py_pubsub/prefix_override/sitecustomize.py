import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aprig/Proyectos_Máster/middleware-master/ros2/ros2_ws/install/py_pubsub'
