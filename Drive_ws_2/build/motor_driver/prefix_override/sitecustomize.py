import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gardentech/Arduino_python_serial/Drive_ws/install/motor_driver'
