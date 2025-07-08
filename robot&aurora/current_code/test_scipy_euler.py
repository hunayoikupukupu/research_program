import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R

# テストデータを生成する関数
def test_euler_rotation(roll, pitch, yaw):
    """
    入力されたEuler角度をRotationオブジェクトに変換し、
    再度Euler角度として取得して比較する
    """
    # 入力角度からRotationオブジェクトを作成
    rotation = R.from_euler('zyx', [yaw, pitch, roll], degrees=True)
    
    # Rotationオブジェクトから角度を取得
    output_angles = rotation.as_euler('zyx', degrees=True)
    output_yaw, output_pitch, output_roll = output_angles
    
    return output_roll, output_pitch, output_yaw

# テストケースを生成
def generate_test_cases():
    test_cases = []
    
    # 0を中心とした対称的な角度の組み合わせをテスト
    # 特にpitch=±90度付近を重点的にテスト
    
    # 基本的なテストケース（0を中心に対称）
    basic_angles = [-180, -135, -90, -45, -30, -15, 0, 15, 30, 45, 90, 135, 180]
    for roll in basic_angles:
        for pitch in basic_angles:
            for yaw in basic_angles:
                test_cases.append((roll, pitch, yaw))
    
    # pitch=±90度付近の詳細テスト（0を中心に対称）
    detailed_angles = list(range(-180, 181, 20))  # -180から180まで20度刻み
    pitch_critical = [-90.5, -90.1, -90.0, -89.9, -89.5, 89.5, 89.9, 90.0, 90.1, 90.5]
    
    for roll in detailed_angles:
        for pitch in pitch_critical:
            for yaw in detailed_angles:
                test_cases.append((roll, pitch, yaw))
    
    # さらに細かい対称テスト
    fine_angles = list(range(-150, 151, 10))  # -150から150まで10度刻み
    for roll in fine_angles[::3]:  # 3つおきに取る
        for pitch in [-120, -90, -60, -30, 0, 30, 60, 90, 120]:
            for yaw in fine_angles[::3]:
                test_cases.append((roll, pitch, yaw))
    
    # 重複を除去
    test_cases = list(set(test_cases))
    
    return test_cases

# CSVデータを作成
def create_csv_data():
    test_cases = generate_test_cases()
    
    data = []
    for input_roll, input_pitch, input_yaw in test_cases:
        try:
            output_roll, output_pitch, output_yaw = test_euler_rotation(input_roll, input_pitch, input_yaw)
            
            # 角度を正規化（-180から180の範囲）
            def normalize_angle(angle):
                while angle > 180:
                    angle -= 360
                while angle <= -180:
                    angle += 360
                return angle
            
            output_roll = normalize_angle(output_roll)
            output_pitch = normalize_angle(output_pitch)
            output_yaw = normalize_angle(output_yaw)
            
            # 入力と出力が等しいかチェック（小数点以下1桁で比較）
            roll_equal = abs(input_roll - output_roll) < 0.1
            pitch_equal = abs(input_pitch - output_pitch) < 0.1
            yaw_equal = abs(input_yaw - output_yaw) < 0.1
            all_equal = roll_equal and pitch_equal and yaw_equal
            
            data.append({
                'input_roll': input_roll,
                'input_pitch': input_pitch,
                'input_yaw': input_yaw,
                'output_roll': round(output_roll, 3),
                'output_pitch': round(output_pitch, 3),
                'output_yaw': round(output_yaw, 3),
                'roll_equal': roll_equal,
                'pitch_equal': pitch_equal,
                'yaw_equal': yaw_equal,
                'all_equal': all_equal,
                'pitch_ge_90': abs(input_pitch) >= 90
            })
        except Exception as e:
            print(f"Error with angles ({input_roll}, {input_pitch}, {input_yaw}): {e}")
    
    return pd.DataFrame(data)

# メイン処理
if __name__ == "__main__":
    # CSVデータを生成
    df = create_csv_data()
    
    # CSVファイルに保存
    df.to_csv('euler_rotation_test.csv', index=False)
    
    print(f"総テストケース数: {len(df)}")
    print(f"CSVファイル 'euler_rotation_test.csv' を作成しました。")
    
    print(f"\n=== データ分布の確認 ===")
    print("Roll角度の分布:")
    print(f"  負の値: {len(df[df['input_roll'] < 0])}")
    print(f"  ゼロ: {len(df[df['input_roll'] == 0])}")
    print(f"  正の値: {len(df[df['input_roll'] > 0])}")
    
    print("Pitch角度の分布:")
    print(f"  負の値: {len(df[df['input_pitch'] < 0])}")
    print(f"  ゼロ: {len(df[df['input_pitch'] == 0])}")
    print(f"  正の値: {len(df[df['input_pitch'] > 0])}")
    
    print("Yaw角度の分布:")
    print(f"  負の値: {len(df[df['input_yaw'] < 0])}")
    print(f"  ゼロ: {len(df[df['input_yaw'] == 0])}")
    print(f"  正の値: {len(df[df['input_yaw'] > 0])}")
    
    # 仮説の検証
    print("\n=== 仮説の検証 ===")
    
    # |pitch| >= 90の場合（正負両方を考慮）
    pitch_ge_90 = df[abs(df['input_pitch']) >= 90]
    print(f"|pitch| >= 90のケース数: {len(pitch_ge_90)}")
    print(f"|pitch| >= 90で input != output のケース数: {len(pitch_ge_90[pitch_ge_90['all_equal'] == False])}")
    if len(pitch_ge_90) > 0:
        print(f"|pitch| >= 90で input != output の割合: {len(pitch_ge_90[pitch_ge_90['all_equal'] == False]) / len(pitch_ge_90) * 100:.1f}%")
    
    # |pitch| < 90の場合
    pitch_lt_90 = df[abs(df['input_pitch']) < 90]
    print(f"\n|pitch| < 90のケース数: {len(pitch_lt_90)}")
    print(f"|pitch| < 90で input != output のケース数: {len(pitch_lt_90[pitch_lt_90['all_equal'] == False])}")
    if len(pitch_lt_90) > 0:
        print(f"|pitch| < 90で input != output の割合: {len(pitch_lt_90[pitch_lt_90['all_equal'] == False]) / len(pitch_lt_90) * 100:.1f}%")
    
    # 詳細な分析
    print("\n=== 詳細分析 ===")
    print("|pitch| >= 90で input != output の例:")
    examples = pitch_ge_90[pitch_ge_90['all_equal'] == False].head(10)
    for _, row in examples.iterrows():
        print(f"Input: ({row['input_roll']}, {row['input_pitch']}, {row['input_yaw']}) "
              f"-> Output: ({row['output_roll']}, {row['output_pitch']}, {row['output_yaw']})")
    
    print(f"\n仮説「input_pitchの絶対値が90以上だと、input != output」は ")
    if len(pitch_ge_90) > 0 and len(pitch_ge_90[pitch_ge_90['all_equal'] == False]) > len(pitch_ge_90) * 0.5:
        print("概ね正しいと言えます。")
    else:
        print("必ずしも正しくありません。")