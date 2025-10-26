import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aprig/Proyectos_Máster/middleware-master/ros2/ros2_ws/py_pubsub/install/py_pubsub'
