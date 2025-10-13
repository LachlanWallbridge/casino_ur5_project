import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lachlan/diceCV/diceCVros2/install/cv_subscriber'
