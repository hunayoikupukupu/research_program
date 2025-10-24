import time
from xarm.wrapper import XArmAPI
from sksurgerynditracker.nditracker import NDITracker

def initialize_robot(ip='192.168.1.155'):
    """ロボットアームの初期化"""
    arm = XArmAPI(ip)
    arm.connect()
    arm.clean_warn()
    arm.clean_error()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    return arm

def initialize_aurora(port='COM3'):
    """オーロラトラッカーの初期化"""
    aurora = NDITracker(
        {
            "tracker type": "aurora",
            "serial port": port,
            "use quaternions": True,
        }
    )
    aurora.start_tracking()
    time.sleep(3)  # トラッキング開始を待つ
    return aurora