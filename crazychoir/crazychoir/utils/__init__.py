# Change this path to your crazyflie-firmware folder
import sys
sys.path.append('/PATH/TO/crazyflie-firmware')

try:
    import cffirmware
except ImportError:
    raise ImportError('Set your crazyflie-firmware folder path in crazychoir/utils/__init__.py')
