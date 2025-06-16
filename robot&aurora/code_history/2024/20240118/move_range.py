# https://github.com/xArm-Developer/xArm-Python-SDK
from xarm.wrapper import XArmAPI

import time

#robot初期設定
arm = XArmAPI('192.168.1.155')
arm.connect()
arm.clean_warn()
arm.clean_error()
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

arm.set_position(x=-50, y=-240, z=200, wait=True)
arm.set_position(x=70, relative=True, wait=True)
arm.set_position(y=-160, relative=True, wait=True)
arm.set_position(x=-70, relative=True, wait=True)
arm.set_position(z=-160, relative=True, wait=True)
arm.set_position(y=160, relative=True, wait=True)
arm.set_position(x=70, relative=True, wait=True)
arm.set_position(y=-160, relative=True, wait=True)
arm.set_position(x=-70, relative=True, wait=True)

arm.disconnect()