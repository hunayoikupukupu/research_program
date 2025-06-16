import numpy as np
import csv
import time

from os.path import dirname, abspath
import sys
parent_dir = dirname(dirname(abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from utils.probe import generate_probe
from utils.robot_pos import generate_robot
from utils.transformation_matrix import findTransformation
from utils.posture import findQuatanion

# https://github.com/SciKit-Surgery/scikit-surgerynditracker/tree/master
from sksurgerynditracker.nditracker import NDITracker

# https://github.com/xArm-Developer/xArm-Python-SDK
from xarm.wrapper import XArmAPI

# ロボットアームで理想状態に移動

#robot初期設定
arm = XArmAPI('192.168.1.155')
arm.connect()
arm.clean_warn()
arm.clean_error()
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

#aurora初期設定
aurora = NDITracker(
    {
        "tracker type": "aurora",
        "serial port": "COM3",
        "use quaternions": True,
    }
)
time.sleep(2)
#トラッキング開始
aurora.start_tracking()
time.sleep(2)

probes = generate_probe(aurora.get_frame())
aurora_pos_x_after = probes[0].pos.x
aurora_pos_y_after = probes[0].pos.y
aurora_pos_z_after = probes[0].pos.z
aurora_quat_x_after = probes[0].quat.x
aurora_quat_y_after = probes[0].quat.y
aurora_quat_z_after = probes[0].quat.z
aurora_quat_w_after = probes[0].quat.w
aurora_quality_after = probes[0].quality

print("aurora_pos_x_after:", np.round(aurora_pos_x_after, 4))
print("aurora_pos_y_after:", np.round(aurora_pos_y_after, 4))
print("aurora_pos_z_after:", np.round(aurora_pos_z_after, 4))
print("aurora_quat_x_after:", np.round(aurora_quat_x_after, 4))
print("aurora_quat_y_after:", np.round(aurora_quat_y_after, 4))
print("aurora_quat_z_after:", np.round(aurora_quat_z_after, 4))
print("aurora_quat_w_after:", np.round(aurora_quat_w_after, 4))

# クォータニオンをオイラー角に変換

# aurora_pos_xとaurora_pos_x_after、aurora_quat_xとaurora_quat_x_afterを比較

# サンプルのデータを取得するプログラムを作成して、目標のところまでやりましょう