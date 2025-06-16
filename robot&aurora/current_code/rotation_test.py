import numpy as np
from relative_orientation_solver import main  # main関数をimport

# テスト用クォータニオンリスト（様々な回転パターンを網羅）
test_quaternions = [
    # 基本回転（単位・180度回転）
    [0, 0, 0, 1],          # 単位クォータニオン（回転なし）
    [1, 0, 0, 0],          # 180度回転 (X軸)
    [0, 1, 0, 0],          # 180度回転 (Y軸)
    [0, 0, 1, 0],          # 180度回転 (Z軸)
    
    # 90度回転（各軸）
    [0.7071, 0, 0, 0.7071],      # 90度回転 (X軸)
    [0, 0.7071, 0, 0.7071],      # 90度回転 (Y軸)
    [0, 0, 0.7071, 0.7071],      # 90度回転 (Z軸)
    [-0.7071, 0, 0, 0.7071],     # -90度回転 (X軸)
    [0, -0.7071, 0, 0.7071],     # -90度回転 (Y軸)
    [0, 0, -0.7071, 0.7071],     # -90度回転 (Z軸)
    
    # 45度回転（各軸）
    [0.3827, 0, 0, 0.9239],      # 45度回転 (X軸)
    [0, 0.3827, 0, 0.9239],      # 45度回転 (Y軸)
    [0, 0, 0.3827, 0.9239],      # 45度回転 (Z軸)
    [-0.3827, 0, 0, 0.9239],     # -45度回転 (X軸)
    [0, -0.3827, 0, 0.9239],     # -45度回転 (Y軸)
    [0, 0, -0.3827, 0.9239],     # -45度回転 (Z軸)
    
    # 30度回転（各軸）
    [0.2588, 0, 0, 0.9659],      # 30度回転 (X軸)
    [0, 0.2588, 0, 0.9659],      # 30度回転 (Y軸)
    [0, 0, 0.2588, 0.9659],      # 30度回転 (Z軸)
    
    # 複合回転（2軸組み合わせ）
    [-0.7071, 0.7071, 0, 0],     # 180度回転 (XY軸)
    [0.7071, 0, 0.7071, 0],      # 180度回転 (XZ軸)
    [0, 0.7071, 0.7071, 0],      # 180度回転 (YZ軸)
    [0.5, 0.5, 0, 0.7071],       # X90度+Y90度
    [0.5, 0, 0.5, 0.7071],       # X90度+Z90度
    [0, 0.5, 0.5, 0.7071],       # Y90度+Z90度
    
    # 複合回転（3軸組み合わせ）
    [0.5, 0.5, 0.5, 0.5],        # 120度回転 (XYZ等方)
    [0.4619, 0.1913, 0.1913, 0.8536],  # X30度+Y30度+Z30度
    [0.6830, 0.1830, 0.1830, 0.6830],  # X45度+Y45度+Z45度
    [0.3536, 0.3536, 0.3536, 0.7454],  # X45度+Y45度+Z30度
    [0.2706, 0.2706, 0.6533, 0.6533],  # X30度+Y30度+Z90度
    
    # 小角度回転（高精度テスト用）
    [0.0872, 0, 0, 0.9962],      # 10度回転 (X軸)
    [0, 0.0872, 0, 0.9962],      # 10度回転 (Y軸)
    [0, 0, 0.0872, 0.9962],      # 10度回転 (Z軸)
    [0.0436, 0.0436, 0, 0.9981], # X5度+Y5度
    
    # 大角度回転
    [0.9659, 0, 0, 0.2588],      # 150度回転 (X軸)
    [0, 0.9659, 0, 0.2588],      # 150度回転 (Y軸)
    [0, 0, 0.9659, 0.2588],      # 150度回転 (Z軸)
    
    # ランダム的な複合回転
    [0.3015, 0.4520, 0.6030, 0.6030],  # 複雑な組み合わせ1
    [0.1826, 0.3651, 0.5477, 0.7303],  # 複雑な組み合わせ2
    [0.4082, 0.4082, 0.4082, 0.7071],  # 対称的な複合回転
]

# 対応するオイラー角配列（固定軸回転XYZ順、度単位）
test_euler_angles = [
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

point_sensor_from_aurora = np.array([70.71068, -141.42136, 135.0])

# main関数の他の引数
x_range = (250, 350)  # X座標の範囲 (開始値, 終了値)
y_range = (-50, 50)   # Y座標の範囲 (開始値, 終了値)
z_range = (75, 175)   # Z座標の範囲 (開始値, 終了値)
divisions = 2         # 分割数（各軸divisions分割でdivisions^3領域）
data_file = 'robot&aurora/current_code/calibration_data/pose_R0-0-45_T100-0-0_ARM180-0-0_SEN0-0-45_n0_qn0.csv'  # データファイルのパス

print(f"\n=== テスト開始: {len(test_quaternions)}個のクォータニオンパターン ===")

# 変換前後の値を保存するリスト
results = []

for i, (quat, euler) in enumerate(zip(test_quaternions, test_euler_angles)):
    print(f"処理中... {i+1}/{len(test_quaternions)}")
    
    # main関数が5つ返す: sensor_point_from_robot, sensor_euler_from_robot, sensor_quat_from_robot, arm_euler_from_robot, arm_quat_from_robot
    sensor_point_from_robot, sensor_euler_from_robot, sensor_quat_from_robot, arm_euler_from_robot, arm_quat_from_robot = main(
        x_range=x_range,
        y_range=y_range,
        z_range=z_range,
        divisions=divisions,
        data_file=data_file,
        input_point=point_sensor_from_aurora.tolist(),
        input_quaternion=quat
    )
    results.append({
        "input_point": point_sensor_from_aurora.tolist(),
        "input_quaternion": quat,
        "input_euler": euler,
        "sensor_point_from_robot": sensor_point_from_robot,
        "sensor_euler_from_robot": sensor_euler_from_robot,
        "sensor_quat_from_robot": sensor_quat_from_robot,
        "arm_euler_from_robot": arm_euler_from_robot,
        "arm_quat_from_robot": arm_quat_from_robot
    })

print("\n検証完了")

print("\n=== 入力値と変換後の値一覧 ===")
for i, res in enumerate(results):
    print(f"\n--- Test Pattern {i+1:2d} ---")
    
    # 変換後の出力をカンマ区切りで整形
    sensor_point_str = ", ".join([f"{v:8.4f}" for v in res['sensor_point_from_robot']])
    sensor_euler_str = ", ".join([f"{v:8.4f}" for v in res['sensor_euler_from_robot']])
    sensor_quat_str = ", ".join([f"{v:8.4f}" for v in res['sensor_quat_from_robot']])
    arm_euler_str = ", ".join([f"{v:8.4f}" for v in res['arm_euler_from_robot']])
    arm_quat_str = ", ".join([f"{v:8.4f}" for v in res['arm_quat_from_robot']])
    
    # 入力値の整形
    input_point_str = ", ".join([f"{v:8.4f}" for v in res['input_point']])
    input_quat_str = ", ".join([f"{v:8.4f}" for v in res['input_quaternion']])
    input_euler_str = ", ".join([f"{v:6.1f}" for v in res['input_euler']])

    print(f"変換前(Sensor_from_Aurora):")
    print(f"　　座標       [{input_point_str}]")
    print(f"　　オイラー角 [{input_euler_str}] (deg)")
    print(f"　　クォータニオン [{input_quat_str}]")
    print(f"変換後(Sensor_from_Robot):")
    print(f"　　座標       [{sensor_point_str}]")
    print(f"　　オイラー角 [{sensor_euler_str}] (deg)")
    print(f"　　クォータニオン [{sensor_quat_str}]")
    print(f"変換後(Arm_from_Robot):")
    print(f"　　オイラー角 [{arm_euler_str}] (deg)")
    print(f"　　クォータニオン [{arm_quat_str}]")

print(f"\n=== 全{len(test_quaternions)}パターンのテスト完了 ===")