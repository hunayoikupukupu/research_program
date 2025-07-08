import numpy as np
from scipy.spatial.transform import Rotation as R
import pandas as pd
import os
from datetime import datetime

def rotation_matrix_fixed_XYZ_deg(roll_deg, pitch_deg, yaw_deg):
    """固定軸XYZ回転でのオイラー角から回転行列を計算"""
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

def verify_csv_transformations(csv_file_path, tolerance=0.001):
    """
    CSVファイルから変換結果を読み込んで検証する
    
    Args:
        csv_file_path (str): CSVファイルのパス
        tolerance (float): 許容誤差（デフォルト: 0.001）
    
    Returns:
        tuple: (verification_results, summary_stats)
    """
    
    # CSVファイルの存在確認
    if not os.path.exists(csv_file_path):
        raise FileNotFoundError(f"CSVファイルが見つかりません: {csv_file_path}")
    
    # CSVファイルを読み込み
    print(f"CSVファイルを読み込み中: {csv_file_path}")
    df = pd.read_csv(csv_file_path)
    
    print(f"読み込みデータ数: {len(df)}行")
    print(f"許容誤差: {tolerance}")
    print("="*60)
    
    # 変換用の回転行列を事前に計算
    # Aurora→Robot変換
    R_aurora_to_robot = R.from_euler('zyx', [167, -55, 20], degrees=True).as_matrix()
    
    # Sensor→Arm変換
    R_sensor_to_arm = R.from_euler('zyx', [-130, -5, 11], degrees=True).as_matrix()
    
    # 結果を格納するリスト
    verification_results = []
    
    # 各行のデータを処理
    for index, row in df.iterrows():
        test_pattern = int(row['Test_Pattern'])
        
        # 入力データの抽出
        aurora_euler = [
            row['Sensor_from_Aurora_Euler_X'],
            row['Sensor_from_Aurora_Euler_Y'],
            row['Sensor_from_Aurora_Euler_Z']
        ]
        
        robot_euler = [
            row['Sensor_from_Robot_Euler_X'],
            row['Sensor_from_Robot_Euler_Y'],
            row['Sensor_from_Robot_Euler_Z']
        ]
        
        arm_euler = [
            row['Arm_from_Robot_Euler_X'],
            row['Arm_from_Robot_Euler_Y'],
            row['Arm_from_Robot_Euler_Z']
        ]
        
        print(f"\n=== Test Pattern {test_pattern}: Aurora[{aurora_euler[0]:.1f}, {aurora_euler[1]:.1f}, {aurora_euler[2]:.1f}] ===")

        aurora_euler_ypr = [aurora_euler[2], aurora_euler[1], aurora_euler[0]]
        robot_euler_ypr = [robot_euler[2], robot_euler[1], robot_euler[0]]
        arm_euler_ypr = [arm_euler[2], arm_euler[1], arm_euler[0]]

        # 各座標系の回転行列を計算
        R_aurora = R.from_euler('zyx', aurora_euler_ypr, degrees=True).as_matrix()
        R_robot_actual = R.from_euler('zyx', robot_euler_ypr, degrees=True).as_matrix()
        R_arm_actual = R.from_euler('zyx', arm_euler_ypr, degrees=True).as_matrix()

        # 1. Aurora→Robot変換の検証
        R_robot_calculated = R_aurora_to_robot @ R_aurora
        aurora_to_robot_match = np.allclose(R_robot_calculated, R_robot_actual, atol=tolerance)
        max_error_aurora_robot = np.max(np.abs(R_robot_calculated - R_robot_actual))
        
        if aurora_to_robot_match:
            print(f"Aurora→Robot変換: 一致 (最大誤差: {max_error_aurora_robot:.6f})")
            aurora_to_robot_status = "一致"
        else:
            print(f"Aurora→Robot変換: 不一致 (最大誤差: {max_error_aurora_robot:.6f})")
            aurora_to_robot_status = "不一致"
            
            # 詳細な差分を表示
            print("  計算結果のオイラー角:")
            calculated_euler = R.from_matrix(R_robot_calculated).as_euler('zyx', degrees=True)
            print(f"    [{calculated_euler[2]:.4f}, {calculated_euler[1]:.4f}, {calculated_euler[0]:.4f}]")
            print("  実際の結果のオイラー角:")
            print(f"    [{robot_euler[0]:.4f}, {robot_euler[1]:.4f}, {robot_euler[2]:.4f}]")
        
        # 2. Sensor→Arm変換の検証
        R_arm_calculated = R_sensor_to_arm @ R_robot_actual
        sensor_to_arm_match = np.allclose(R_arm_calculated, R_arm_actual, atol=tolerance)
        max_error_sensor_arm = np.max(np.abs(R_arm_calculated - R_arm_actual))
        
        if sensor_to_arm_match:
            print(f"Sensor→Arm変換: 一致 (最大誤差: {max_error_sensor_arm:.6f})")
            sensor_to_arm_status = "一致"
        else:
            print(f"Sensor→Arm変換: 不一致 (最大誤差: {max_error_sensor_arm:.6f})")
            sensor_to_arm_status = "不一致"
            
            # 詳細な差分を表示
            print("  計算結果のオイラー角:")
            calculated_euler = R.from_matrix(R_arm_calculated).as_euler('zyx', degrees=True)
            print(f"    [{calculated_euler[2]:.4f}, {calculated_euler[1]:.4f}, {calculated_euler[0]:.4f}]")
            print("  実際の結果のオイラー角:")
            print(f"    [{arm_euler[0]:.4f}, {arm_euler[1]:.4f}, {arm_euler[2]:.4f}]")
        
        # 結果を記録
        verification_results.append({
            'Test_Pattern': test_pattern,
            'Sensor_from_Aurora_Roll': aurora_euler[0],
            'Sensor_from_Aurora_Pitch': aurora_euler[1],
            'Sensor_from_Aurora_Yaw': aurora_euler[2],
            'Sensor_from_Robot_Roll': robot_euler[0],
            'Sensor_from_Robot_Pitch': robot_euler[1],
            'Sensor_from_Robot_Yaw': robot_euler[2],
            'Arm_from_Robot_Roll': arm_euler[0],
            'Arm_from_Robot_Pitch': arm_euler[1],
            'Arm_from_Robot_Yaw': arm_euler[2],
            'Aurora_to_Robot_Status': aurora_to_robot_status,
            'Sensor_to_Arm_Status': sensor_to_arm_status,
            'Aurora_to_Robot_Max_Error': max_error_aurora_robot,
            'Sensor_to_Arm_Max_Error': max_error_sensor_arm
        })
    
    return verification_results

def save_verification_results(verification_results, tolerance):
    """検証結果をCSVファイルに保存"""
    
    # DataFrameに変換
    df_results = pd.DataFrame(verification_results)
    
    # CSVファイル名を生成（タイムスタンプ付き）
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"transform_verification_results_{timestamp}.csv"
    
    # CSVファイルに保存
    df_results.to_csv(csv_filename, index=False, float_format='%.6f')
    
    print(f"\n=== 検証結果を保存しました ===")
    print(f"ファイル名: {csv_filename}")
    print(f"保存場所: {os.path.abspath(csv_filename)}")
    
    return csv_filename, df_results

def print_summary_statistics(verification_results, tolerance):
    """サマリー統計を表示"""
    
    total_cases = len(verification_results)
    aurora_robot_matches = sum(1 for r in verification_results if r['Aurora_to_Robot_Status'] == '一致')
    sensor_arm_matches = sum(1 for r in verification_results if r['Sensor_to_Arm_Status'] == '一致')
    
    aurora_robot_errors = [r['Aurora_to_Robot_Max_Error'] for r in verification_results]
    sensor_arm_errors = [r['Sensor_to_Arm_Max_Error'] for r in verification_results]
    
    print(f"\n=== サマリー統計（許容誤差: {tolerance}） ===")
    print(f"総ケース数: {total_cases}")
    print(f"Aurora→Robot変換 一致: {aurora_robot_matches}件 ({aurora_robot_matches/total_cases*100:.1f}%)")
    print(f"Aurora→Robot変換 不一致: {total_cases-aurora_robot_matches}件 ({(total_cases-aurora_robot_matches)/total_cases*100:.1f}%)")
    print(f"Sensor→Arm変換 一致: {sensor_arm_matches}件 ({sensor_arm_matches/total_cases*100:.1f}%)")
    print(f"Sensor→Arm変換 不一致: {total_cases-sensor_arm_matches}件 ({(total_cases-sensor_arm_matches)/total_cases*100:.1f}%)")
    
    print(f"\n=== エラー統計 ===")
    print(f"Aurora→Robot変換:")
    print(f"  最大エラー: {max(aurora_robot_errors):.6f}")
    print(f"  平均エラー: {np.mean(aurora_robot_errors):.6f}")
    print(f"  最小エラー: {min(aurora_robot_errors):.6f}")
    
    print(f"Sensor→Arm変換:")
    print(f"  最大エラー: {max(sensor_arm_errors):.6f}")
    print(f"  平均エラー: {np.mean(sensor_arm_errors):.6f}")
    print(f"  最小エラー: {min(sensor_arm_errors):.6f}")

def main():
    """メイン関数"""
    
    # 設定
    tolerance = 0.001  # 許容誤差
    
    # CSVファイルのパスを指定（ユーザー入力または直接指定）
    print("=== CSV変換結果検証プログラム ===")
    
    # CSVファイルの選択
    csv_file_path = input("検証するCSVファイルのパスを入力してください: ").strip()
    
    # ファイルが存在しない場合は、カレントディレクトリの最新ファイルを検索
    if not csv_file_path or not os.path.exists(csv_file_path):
        print("ファイルが指定されていないか、存在しません。")
        print("カレントディレクトリから最新のtransform_test_results_*.csvファイルを検索します...")
        
        # カレントディレクトリのCSVファイルを検索
        csv_files = [f for f in os.listdir('.') if f.startswith('transform_test_results_') and f.endswith('.csv')]
        
        if csv_files:
            # 最新のファイルを選択
            csv_files.sort(reverse=True)  # 日付順でソート
            csv_file_path = csv_files[0]
            print(f"最新ファイルを使用: {csv_file_path}")
        else:
            print("適切なCSVファイルが見つかりません。")
            return
    
    try:
        # 検証実行
        verification_results = verify_csv_transformations(csv_file_path, tolerance)
        
        # 結果を保存
        output_filename, df_results = save_verification_results(verification_results, tolerance)
        
        # サマリー統計を表示
        print_summary_statistics(verification_results, tolerance)
        
        print(f"\n=== 検証完了 ===")
        print(f"詳細結果は {output_filename} をご確認ください。")
        
    except Exception as e:
        print(f"エラーが発生しました: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()