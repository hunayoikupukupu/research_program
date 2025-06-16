from sksurgerynditracker.nditracker import NDITracker

import time

from os.path import dirname, abspath
import sys
parent_dir = dirname(dirname(abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)
    
from utils.probe import generate_probe

#aurora初期設定
aurora = NDITracker(
    {
        "tracker type": "aurora",
        "serial port": "COM3",
        "use quaternions": True,
    }
)

aurora.start_tracking()
time.sleep(3)

ave_x = 0
ave_y = 0
ave_z = 0

#aurora座標系上でセンサー位置を出力
for i in range(3): 
    probes = generate_probe(aurora.get_frame())
    print("aurora_pos:" + probes[1].pos.to_str())
    ave_x += probes[1].pos.x/3
    ave_y += probes[1].pos.y/3
    ave_z += probes[1].pos.z/3



print(ave_x)
print(ave_y)
print(ave_z)

aurora.stop_tracking()
aurora.close()