import numpy as np
from scipy.spatial.transform import Rotation as R
import pandas as pd
import csv

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
Sensor_from_aurora = [
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

# センサー出力リスト（オイラー角 [roll, pitch, yaw] in degrees）
Sensor_from_robot = [
    # 基本回転（単位・180度回転）
    [0, 0, 45],          # 回転なし
    [180, 0, 45],        # X軸180度
    [180, 0, -135],      # Y軸180度
    [0, 0, -135],        # Z軸180度
    
    # 90度回転（各軸）
    [90, 0, 45],         # X軸90度
    [-45, 90, 0],        # Y軸90度
    [135, 0, 0],         # Z軸90度
    [-90, 0, 45],        # X軸-90度
    [45, -90, 0],        # Y軸-90度
    [0, -90, 0],         # Z軸-90度
    
    # 45度回転（各軸）
    [45.0009, 0, 45],    # X軸45度
    [0, 45.0009, 45],    # Y軸45度
    [0, 0, 90.0009],     # Z軸45度
    [-45.0009, 0, 45],   # X軸-45度
    [0, -45.0009, 45],   # Y軸-45度
    [0, 0, -0.0009],     # Z軸-45度
    
    # 30度回転（各軸）
    [29.9987, 0, 45],    # X軸30度
    [0, 29.9987, 45],    # Y軸30度
    [0, 0, 74.9987],     # Z軸30度
    
    # 複合回転（2軸組み合わせ）
    [-180, 0, -45],      # X180度+Y180度
    [-135, -90, 0],      # X180度+Z180度
    [90, 0, -135],       # Y180度+Z180度
    [90.0008, 45, 90.0005],  # X90度+Y90度
    [54.7359, -30.0003, 99.7359],  # X90度+Z90度
    [45.0005, 45, 135.0008],  # Y90度+Z90度
    
    # 複合回転（3軸組み合わせ）
    [90, 0, 135],        # XYZ各120度（近似）
    [59.1227, 8.4893, 75.0846],  # XYZ各30度
    [90, 0, 74.9985],    # XYZ各45度
    [61.0129, 17.3199, 106.0129],  # X45度+Y45度+Z30度
    [44.9991, 0, 135],   # X30度+Y30度+Z90度
    
    # 小角度回転（高精度テスト用）
    [10.005, 0, 45],     # X軸10度
    [0, 10.005, 45],     # Y軸10度
    [0, 0, 55.005],      # Z軸10度
    [5.0121, 4.993, 45.2187],  # X5度+Y5度
    
    # 大角度回転
    [150.0013, 0, 45],   # X軸150度
    [-180, 29.9987, -135],  # Y軸150度
    [0, 0, -164.9987],   # Z軸150度
    
    # ランダム的な複合回転（近似値）
    [64.5733, 10.2255, 141.4713],  # 複雑な組み合わせ1
    [44.9984, 19.4668, 126.8652],  # 複雑な組み合わせ2
    [69.8913, 14.1259, 114.8913],  # 対称的な複合回転
]

# アーム出力リスト（オイラー角 [roll, pitch, yaw] in degrees）
Arm_from_robot = [
    # 基本回転（単位・180度回転）
    [0, 0, 90],          # 回転なし
    [180, 0, 90],        # X軸180度
    [-180, 0, -90],      # Y軸180度
    [0, 0, -90],         # Z軸180度
    
    # 90度回転（各軸）
    [90, 0, 90],         # X軸90度
    [-90, 90, 0],        # Y軸90度
    [180, -90, 0],       # Z軸90度
    [-90, 0, 90],        # X軸-90度
    [90, -90, 0],        # Y軸-90度
    [0, -90, 0],         # Z軸-90度
    
    # 45度回転（各軸）
    [45.0009, 0, 90],    # X軸45度
    [0, 45.0009, 90],    # Y軸45度
    [0, 0, 135.0009],    # Z軸45度
    [-45.0009, 0, 90],   # X軸-45度
    [0, -45.0009, 90],   # Y軸-45度
    [0, 0, 44.9991],     # Z軸-45度
    
    # 30度回転（各軸）
    [29.9987, 0, 90],    # X軸30度
    [0, 29.9987, 90],    # Y軸30度
    [0, 0, 119.9987],    # Z軸30度
    
    # 複合回転（2軸組み合わせ）
    [-180, 0, 0],        # X180度+Y180度
    [-90, -90, 0],       # X180度+Z180度
    [90, 0, -90],        # Y180度+Z180度
    [90.0008, 45, 135.0005],  # X90度+Y90度
    [54.7359, -30.0003, 144.7359],  # X90度+Z90度
    [45.0005, 45, -179.9992],  # Y90度+Z90度
    
    # 複合回転（3軸組み合わせ）
    [90, 0, 180],        # XYZ各120度（近似）
    [59.1227, 8.4893, 120.0846],  # XYZ各30度
    [90, 0, 119.9985],   # XYZ各45度
    [61.0129, 17.3199, 151.0129],  # X45度+Y45度+Z30度
    [44.9991, 0, 180],   # X30度+Y30度+Z90度
    
    # 小角度回転（高精度テスト用）
    [10.005, 0, 90],     # X軸10度
    [0, 10.005, 90],     # Y軸10度
    [0, 0, 100.005],     # Z軸10度
    [5.0121, 4.993, 90.2187],  # X5度+Y5度
    
    # 大角度回転
    [150.0013, 0, 90],   # X軸150度
    [-180, 29.9987, -90],  # Y軸150度
    [0, 0, -119.9987],   # Z軸150度
    
    # ランダム的な複合回転（近似値）
    [64.5733, 10.2255, -173.5287],  # 複雑な組み合わせ1
    [44.9984, 19.4668, 171.8652],   # 複雑な組み合わせ2
    [69.8913, 14.1259, 159.8913],   # 対称的な複合回転
]

# 結果を格納するリスト
results = []

R_manual_sfa = []
R_scipy_sfa = []
R_manual_sfr = []
R_scipy_sfr = []
R_manual_afr = []
R_scipy_afr = []

# 入力値を配列に格納
for i, angles in enumerate(Sensor_from_aurora):
    roll_deg, pitch_deg, yaw_deg = angles

    R_manual = rotation_matrix_fixed_XYZ_deg(roll_deg, pitch_deg, yaw_deg)
    R_scipy = R.from_euler('xyz', angles, degrees=True).as_matrix()
    R_manual_sfa.append(R_manual)
    R_scipy_sfa.append(R_scipy)

for i, angles in enumerate(Sensor_from_robot):
    roll_deg, pitch_deg, yaw_deg = angles
    R_manual = rotation_matrix_fixed_XYZ_deg(roll_deg, pitch_deg, yaw_deg)
    R_scipy = R.from_euler('xyz', angles, degrees=True).as_matrix()
    R_manual_sfr.append(R_manual)
    R_scipy_sfr.append(R_scipy)

for i, angles in enumerate(Arm_from_robot):
    roll_deg, pitch_deg, yaw_deg = angles
    R_manual = rotation_matrix_fixed_XYZ_deg(roll_deg, pitch_deg, yaw_deg)
    R_scipy = R.from_euler('xyz', angles, degrees=True).as_matrix()
    R_manual_afr.append(R_manual)
    R_scipy_afr.append(R_scipy)

# 適用する回転行列をオイラー角から生成
R_scipy_aur2rob = R.from_euler('xyz', [0, 0, 45], degrees=True).as_matrix()
R_manual_aur2rob = rotation_matrix_fixed_XYZ_deg(0, 0, 45)
R_scipy_sen2arm = R.from_euler('xyz', [0, 0, 45], degrees=True).as_matrix()
R_manual_sen2arm = rotation_matrix_fixed_XYZ_deg(0, 0, 45)

# 配列の長さを記録
length = len(Sensor_from_aurora)

# 初期値を設定
# 許容誤差: 0.001（1mm単位での一致判定）
tolerance = 0.001
all_cases = True

print(f"\n許容誤差: {tolerance} で一致判定を実行します")
print("="*50)

for i in range(length):
    print(f"\n=== ケース{i+1}: [roll={Sensor_from_aurora[i][0]}, pitch={Sensor_from_aurora[i][1]}, yaw={Sensor_from_aurora[i][2]}] ===")

    # auroraからrobotへの変換を適用
    R_sfr_calculated = R_scipy_aur2rob @ R_scipy_sfa[i]
    R_sfr_scipy = R_scipy_sfr[i]

    # R_sfrが一致しているか確認（設定した許容誤差で判定）
    aurora_to_robot_match = np.allclose(R_sfr_calculated, R_sfr_scipy, atol=tolerance)
    max_error_aurora_robot = np.max(np.abs(R_sfr_calculated - R_sfr_scipy))
    if aurora_to_robot_match:
        print(f"AuroraからRobotへの変換: 一致 (最大誤差: {max_error_aurora_robot:.6f})")
        aurora_to_robot_status = "一致"
    else:
        print(f"AuroraからRobotへの変換: 不一致 (最大誤差: {max_error_aurora_robot:.6f})")
        print("計算結果:")
        print(np.round(R_sfr_calculated, 5))
        print("scipy結果:")
        print(np.round(R_sfr_scipy, 5))
        all_cases = False
        aurora_to_robot_status = "不一致"
    
    # sensorからarmへの変換を適用
    R_afr_calculated = R_scipy_sen2arm @ R_scipy_sfr[i]
    R_afr_scipy = R_scipy_afr[i]

    # R_afrが一致しているか確認（設定した許容誤差で判定）
    sensor_to_arm_match = np.allclose(R_afr_calculated, R_afr_scipy, atol=tolerance)
    max_error_sensor_arm = np.max(np.abs(R_afr_calculated - R_afr_scipy))
    if sensor_to_arm_match:
        print(f"SensorからArmへの変換: 一致 (最大誤差: {max_error_sensor_arm:.6f})")
        sensor_to_arm_status = "一致"
    else:
        print(f"SensorからArmへの変換: 不一致 (最大誤差: {max_error_sensor_arm:.6f})")
        print("計算結果:")
        print(np.round(R_afr_calculated, 5))
        print("scipy結果:")
        print(np.round(R_afr_scipy, 5))
        all_cases = False
        sensor_to_arm_status = "不一致"
    
    # 結果をリストに追加
    results.append({
        'ケース番号': i + 1,
        'Aurora_Roll': Sensor_from_aurora[i][0],
        'Aurora_Pitch': Sensor_from_aurora[i][1],
        'Aurora_Yaw': Sensor_from_aurora[i][2],
        'Robot_Roll': Sensor_from_robot[i][0],
        'Robot_Pitch': Sensor_from_robot[i][1],
        'Robot_Yaw': Sensor_from_robot[i][2],
        'Arm_Roll': Arm_from_robot[i][0],
        'Arm_Pitch': Arm_from_robot[i][1],
        'Arm_Yaw': Arm_from_robot[i][2],
        'Aurora→Robot変換': aurora_to_robot_status,
        'Sensor→Arm変換': sensor_to_arm_status,
        'Aurora→Robot_Max_Error': max_error_aurora_robot,
        'Sensor→Arm_Max_Error': max_error_sensor_arm
    })

# 最終結果の出力
if all_cases:
    print(f"\n=== 全てのケースで一致（許容誤差: {tolerance}） ===")
else:
    print(f"\n=== 一部のケースで不一致がありました（許容誤差: {tolerance}） ===")

# DataFrameに変換してCSV出力
df = pd.DataFrame(results)

# CSV出力
csv_filename = 'rotation_matrix_verification_results.csv'
df.to_csv(csv_filename, index=False, encoding='utf-8')
print(f"\n結果を {csv_filename} に出力しました")

# サマリー情報も出力
print(f"\n=== サマリー（許容誤差: {tolerance}） ===")
print(f"総ケース数: {len(results)}")
print(f"Aurora→Robot変換 一致: {sum(1 for r in results if r['Aurora→Robot変換'] == '一致')}件")
print(f"Aurora→Robot変換 不一致: {sum(1 for r in results if r['Aurora→Robot変換'] == '不一致')}件")
print(f"Sensor→Arm変換 一致: {sum(1 for r in results if r['Sensor→Arm変換'] == '一致')}件")
print(f"Sensor→Arm変換 不一致: {sum(1 for r in results if r['Sensor→Arm変換'] == '不一致')}件")

# エラーの統計も出力
aurora_robot_errors = [r['Aurora→Robot_Max_Error'] for r in results]
sensor_arm_errors = [r['Sensor→Arm_Max_Error'] for r in results]

print(f"\nAurora→Robot変換の最大エラー: {max(aurora_robot_errors):.6f}")
print(f"Aurora→Robot変換の平均エラー: {np.mean(aurora_robot_errors):.6f}")
print(f"Sensor→Arm変換の最大エラー: {max(sensor_arm_errors):.6f}")
print(f"Sensor→Arm変換の平均エラー: {np.mean(sensor_arm_errors):.6f}")