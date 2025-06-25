import numpy as np
import pandas as pd
import adaptive_transform  # パッケージ全体をimport
from datetime import datetime
import os

from scipy.spatial.transform import Rotation as R
import numpy as np

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

# オイラー角からクォータニオンに変換
test_quaternions = []
for euler in test_euler_angles:
    r = R.from_euler('xyz', euler, degrees=True)
    test_quaternions.append(r.as_quat().tolist())


# テスト設定
point_sensor_from_aurora = np.array([21.99209,110.29819,188.03366])  # Auroraセンサーからの座標

# main関数の他の引数
x_range = (250, 350)  # X座標の範囲 (開始値, 終了値)
y_range = (-50, 50)   # Y座標の範囲 (開始値, 終了値)
z_range = (75, 175)   # Z座標の範囲 (開始値, 終了値)
divisions = 2         # 分割数（各軸divisions分割でdivisions^3領域）
data_file = 'robot&aurora/current_code/calibration_data/pose_R20--55-167_T100-25--66_ARM-20-99-43_SEN11--5--130_n0_qn0.csv'  # データファイルのパス

print(f"\n=== テスト開始: {len(test_quaternions)}個のクォータニオンパターン ===")

# CSV出力用のデータリスト
csv_data = []

# 変換前後の値を保存するリスト（コンソール出力用）
results = []

for i, (quat, euler) in enumerate(zip(test_quaternions, test_euler_angles)):
    print(f"処理中... {i+1}/{len(test_quaternions)}")
    
    # main関数が5つ返す: sensor_point_from_robot, sensor_euler_from_robot, sensor_quat_from_robot, arm_euler_from_robot, arm_quat_from_robot
    sensor_point_from_robot, sensor_euler_from_robot, sensor_quat_from_robot, arm_euler_from_robot, arm_quat_from_robot = adaptive_transform.main(
        x_range=x_range,
        y_range=y_range,
        z_range=z_range,
        divisions=divisions,
        data_file=data_file,
        input_point=point_sensor_from_aurora.tolist(),
        input_quaternion=quat
    )
    
    # コンソール出力用データ保存
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
    
    # CSV出力用データ作成
    csv_row = {
        'Test_Pattern': i + 1,
        
        # Sensor_from_Aurora (入力データ)
        'Sensor_from_Aurora_X': point_sensor_from_aurora[0],
        'Sensor_from_Aurora_Y': point_sensor_from_aurora[1],
        'Sensor_from_Aurora_Z': point_sensor_from_aurora[2],
        'Sensor_from_Aurora_Euler_X': euler[0],
        'Sensor_from_Aurora_Euler_Y': euler[1],
        'Sensor_from_Aurora_Euler_Z': euler[2],
        'Sensor_from_Aurora_Quat_X': quat[0],
        'Sensor_from_Aurora_Quat_Y': quat[1],
        'Sensor_from_Aurora_Quat_Z': quat[2],
        'Sensor_from_Aurora_Quat_W': quat[3],
        
        # Sensor_from_Robot (変換後データ)
        'Sensor_from_Robot_X': sensor_point_from_robot[0],
        'Sensor_from_Robot_Y': sensor_point_from_robot[1],
        'Sensor_from_Robot_Z': sensor_point_from_robot[2],
        'Sensor_from_Robot_Euler_X': sensor_euler_from_robot[0],
        'Sensor_from_Robot_Euler_Y': sensor_euler_from_robot[1],
        'Sensor_from_Robot_Euler_Z': sensor_euler_from_robot[2],
        'Sensor_from_Robot_Quat_X': sensor_quat_from_robot[0],
        'Sensor_from_Robot_Quat_Y': sensor_quat_from_robot[1],
        'Sensor_from_Robot_Quat_Z': sensor_quat_from_robot[2],
        'Sensor_from_Robot_Quat_W': sensor_quat_from_robot[3],
        
        # Arm_from_Robot (変換後データ)
        'Arm_from_Robot_Euler_X': arm_euler_from_robot[0],
        'Arm_from_Robot_Euler_Y': arm_euler_from_robot[1],
        'Arm_from_Robot_Euler_Z': arm_euler_from_robot[2],
        'Arm_from_Robot_Quat_X': arm_quat_from_robot[0],
        'Arm_from_Robot_Quat_Y': arm_quat_from_robot[1],
        'Arm_from_Robot_Quat_Z': arm_quat_from_robot[2],
        'Arm_from_Robot_Quat_W': arm_quat_from_robot[3],
    }
    
    csv_data.append(csv_row)

print("\n検証完了")

# CSVファイル出力
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_filename = f"transform_test_results_{timestamp}.csv"

# DataFrameを作成してCSVに保存
df = pd.DataFrame(csv_data)
df.to_csv(csv_filename, index=False, float_format='%.6f')

print(f"\n=== CSV出力完了 ===")
print(f"ファイル名: {csv_filename}")
print(f"保存場所: {os.path.abspath(csv_filename)}")
print(f"データ行数: {len(csv_data)}行")

# コンソールにも簡潔な結果を表示（オプション）
print_console = input("\nコンソールにも詳細結果を表示しますか？ (y/n): ").lower() == 'y'

if print_console:
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