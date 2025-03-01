import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/amogh1216/Documents/VSCodeProjects/tr-autonomy-2-amogh1216/src/spinnyrobot/install/spinnyrobot'
