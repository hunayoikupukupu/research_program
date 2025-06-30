import numpy as np
from scipy.spatial.transform import Rotation as R

def rotation_matrix_fixed_XYZ_deg(roll_deg, pitch_deg, yaw_deg):
    roll = np.deg2rad(roll_deg)
    pitch = np.deg2rad(pitch_deg)
    yaw = np.deg2rad(yaw_deg)

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    return Rz @ Ry @ Rx

# 入力リスト（オイラー角 [roll, pitch, yaw] in degrees）
euler_angles_deg_list = [
    # 基本回転（単位・180度回転）
    [0, 0, 0],           # 回転なし
    [180, 0, 0],         # X軸180度
    [0, 180, 0],         # Y軸180度
    [0, 0, 180],         # Z軸180度
    
    # 90度回転（各軸）
    [90, 0, 0],          # X軸90度
    [0, 90, 0],          # Y軸90度
    [0, 0, 90],          # Z軸90度
    [-90, 0, 0],         # X軸-90度
    [0, -90, 0],         # Y軸-90度
    [0, 0, -90],         # Z軸-90度
    
    # 45度回転（各軸）
    [45, 0, 0],          # X軸45度
    [0, 45, 0],          # Y軸45度
    [0, 0, 45],          # Z軸45度
    [-45, 0, 0],         # X軸-45度
    [0, -45, 0],         # Y軸-45度
    [0, 0, -45],         # Z軸-45度
    
    # 30度回転（各軸）
    [30, 0, 0],          # X軸30度
    [0, 30, 0],          # Y軸30度
    [0, 0, 30],          # Z軸30度
    
    # 複合回転（2軸組み合わせ）
    [180, 180, 0],       # X180度+Y180度
    [180, 0, 180],       # X180度+Z180度
    [0, 180, 180],       # Y180度+Z180度
    [90, 90, 0],         # X90度+Y90度
    [90, 0, 90],         # X90度+Z90度
    [0, 90, 90],         # Y90度+Z90度
    
    # 複合回転（3軸組み合わせ）
    [120, 120, 120],     # XYZ各120度（近似）
    [30, 30, 30],        # XYZ各30度
    [45, 45, 45],        # XYZ各45度
    [45, 45, 30],        # X45度+Y45度+Z30度
    [30, 30, 90],        # X30度+Y30度+Z90度
    
    # 小角度回転（高精度テスト用）
    [10, 0, 0],          # X軸10度
    [0, 10, 0],          # Y軸10度
    [0, 0, 10],          # Z軸10度
    [5, 5, 0],           # X5度+Y5度
    
    # 大角度回転
    [150, 0, 0],         # X軸150度
    [0, 150, 0],         # Y軸150度
    [0, 0, 150],         # Z軸150度
    
    # ランダム的な複合回転（近似値）
    [60, 80, 100],       # 複雑な組み合わせ1
    [30, 60, 90],        # 複雑な組み合わせ2
    [60, 60, 60],        # 対称的な複合回転
]

for i, angles in enumerate(euler_angles_deg_list):
    roll_deg, pitch_deg, yaw_deg = angles

    R_manual = rotation_matrix_fixed_XYZ_deg(roll_deg, pitch_deg, yaw_deg)
    R_scipy = R.from_euler('xyz', angles, degrees=True).as_matrix()

    print(f"\n=== ケース{i+1}: [roll={roll_deg}, pitch={pitch_deg}, yaw={yaw_deg}] ===")
    print("NumPy手動計算の回転行列:")
    print(np.round(R_manual, 5))
    print("scipy Rotationの回転行列:")
    print(np.round(R_scipy, 5))
    print("一致するか？ →", np.allclose(R_manual, R_scipy))
