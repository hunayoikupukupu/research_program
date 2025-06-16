import numpy as np
from scipy.spatial.transform import Rotation as R
import sys
from regionwise_pose_estimation import build_coordinate_transformation_model, transform_pose


euler_test_data = np.array([
    [0, 0, 0],                  # 原点（初期姿勢）
    [90, 0, 0],                 # ロール90度
    [180, 0, 0],                # ロール180度
    [-90, 0, 0],                # ロール-90度
    [0, 90, 0],                 # ピッチ90度（特異点に近い）
    [0, -90, 0],                # ピッチ-90度（特異点に近い）
    [0, 45, 0],                 # ピッチ45度
    [0, -45, 0],                # ピッチ-45度
    [0, 0, 90],                 # ヨー90度
    [0, 0, 180],                # ヨー180度
    [0, 0, -90],                # ヨー-90度
    [45, 45, 45],               # 均等な回転（45度ずつ）
    [90, 45, 0],                # ロール90度+ピッチ45度
    [0, 45, 90],                # ピッチ45度+ヨー90度
    [90, 0, 90],                # ロール90度+ヨー90度
    [90, 45, 90],               # ロール90度+ピッチ45度+ヨー90度
    [180, 90, 180],             # すべての角度が極端な値
    [60, 30, 45],               # 一般的な回転の組み合わせ
    [30, 60, 120],              # 別の組み合わせ
    [120, -30, 60]              # 正負混合の組み合わせ
])

quaternion_test_data = R.from_euler('XYZ', euler_test_data, degrees=True).as_quat()

x_range=(250, 350)                    # X座標の範囲 (開始値, 終了値)
y_range=(-50, 50)                     # Y座標の範囲 (開始値, 終了値)
z_range=(75, 175)                     # Z座標の範囲 (開始値, 終了値)
divisions=2                           # 分割数（各軸divisions分割でdivisions^3領域）
data_file='robot&aurora/current_code/calibration_data/aurora_robot_pose_log_r0_p0_y90_static.csv'  # データファイルのパス
input_point=[30.000000000000068,310.0,115.0]

# 変換モデルの構築
R_matrices, t_vectors, transformer = build_coordinate_transformation_model(
    x_range, y_range, z_range, divisions, data_file
)

# 変換モデルの構築に失敗した場合は終了
if R_matrices is None:
    sys.exit()

# テストデータを使用して変換を実行
for i in range(len(quaternion_test_data)):
    # 結果番号を表示
    print(f"Test {i+1}：オイラー角 [{euler_test_data[i][0]}, {euler_test_data[i][1]}, {euler_test_data[i][2]}]")   
    input_quaternion = quaternion_test_data[i]

    # 入力が提供されている場合は変換を実行
    if input_point is not None and input_quaternion is not None:
        transform_pose(input_point, input_quaternion, R_matrices, t_vectors, transformer)
    else:    
        # 例の変換結果を返さない（None）
        sys.exit()    
    

