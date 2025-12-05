import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tkn-yangfukang/HiveMind-Dev/Third-Party-Implementation-Unitree-SDK/go2/unitree-go2-slam-toolbox/install/message_test'
