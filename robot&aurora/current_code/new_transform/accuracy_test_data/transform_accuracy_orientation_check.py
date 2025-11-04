import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import os
import sys

# --- 1. データの読み込みと前処理 (回転ベクトルとして) ---

def load_and_group_data(filepath, grouping_precision=0):
    """
    CSVを読み込み、angle_diffを計算し、グループ化された平均値と
    全体の統計情報（平均、最大、最小）の両方を返します。

    Args:
        filepath (str): CSVファイルへのパス
        grouping_precision (int): グループ化のための丸め精度

    Returns:
        tuple: (df_grouped, overall_stats)
               df_grouped: グループ化・ソートされたDataFrame
               overall_stats: 全体の統計情報を含む辞書
               または (None, None) (エラー時)
    """
    print(f"Loading data from '{filepath}'...")
    if not os.path.exists(filepath):
        print(f"Error: File not found at the specified path: {filepath}")
        return None, None
        
    try:
        df = pd.read_csv(filepath)
    except Exception as e:
        print(f"Error reading data: {e}")
        return None, None

    print("Calculating angle_diff (processing as rotation vectors)...")
    
    # 必要な列のリスト
    required_cols = [
        'robot_rx', 'robot_ry', 'robot_rz',
        'delta_t_robot_norm', 'delta_R_robot_angle',
        'delta_t_aurora_norm', 'delta_R_aurora_angle'
    ]
    
    # 必要な列が存在するかチェック
    missing_cols = [col for col in required_cols if col not in df.columns]
    if missing_cols:
        print(f"Error: Missing required columns in CSV: {missing_cols}")
        return None, None

    # 1. 基準の姿勢をRotationオブジェクトとして定義
    ref_rotvec_deg = np.array([0, 0, 180])
    ref_rotvec_rad = np.radians(ref_rotvec_deg)
    R_ref = Rotation.from_rotvec(ref_rotvec_rad)

    # 2. 各行の姿勢データをRotationオブジェクトとして読み込み
    try:
        robot_rotvecs_deg = df[['robot_rx', 'robot_ry', 'robot_rz']].values
        robot_rotvecs_rad = np.radians(robot_rotvecs_deg)
        R_data = Rotation.from_rotvec(robot_rotvecs_rad)
    except Exception as e:
        print(f"Error creating Rotation object: {e}")
        return None, None

    # 3. 差分回転 R_diff を計算
    R_diff = R_data * R_ref.inv()

    # 4. 差分回転の角度(magnitude)を計算し、度(degree)に変換
    df['angle_diff'] = np.degrees(R_diff.magnitude())
    
    # 5. 【全体の統計情報】を先に計算
    cols_to_analyze = [
        'delta_t_robot_norm', 'delta_R_robot_angle',
        'delta_t_aurora_norm', 'delta_R_aurora_angle'
    ]
    
    overall_stats = {}
    print("\n--- Overall Statistics (Before Grouping) ---")
    for col in cols_to_analyze:
        overall_stats[col] = {
            'mean': df[col].mean(),
            'max': df[col].max(),
            'min': df[col].min()
        }
        # ユーザーの要求通り、テキストで表示
        print(f"[{col}]")
        print(f"  Mean: {overall_stats[col]['mean']:.4f}")
        print(f"  Max:  {overall_stats[col]['max']:.4f}")
        print(f"  Min:  {overall_stats[col]['min']:.4f}")
    print("----------------------------------------------\n")

    # 6. 【グループ化】
    df['angle_diff_group'] = df['angle_diff'].round(grouping_precision)

    # 7. グループ化して平均を計算
    print(f"Grouping by angle_diff (rounded to {grouping_precision} decimal places) and calculating mean...")
    cols_to_average = [
        'delta_t_robot_norm', 'delta_R_robot_angle',
        'delta_t_aurora_norm', 'delta_R_aurora_angle'
    ]
    df_grouped = df.groupby('angle_diff_group')[cols_to_average].mean()

    # 8. 【ソート】
    df_grouped = df_grouped.sort_index(ascending=True)

    print("Preprocessing and grouping complete.")
    return df_grouped, overall_stats


# --- 2. グラフの描画 ---

def plot_delta_graphs(df_grouped, overall_stats, source='robot'):
    """
    グループ化されたDataFrameに基づいて、棒グラフと全体の平均線、
    および最大/最小値をテキストで描画します。

    Args:
        df_grouped (pd.DataFrame): グループ化・ソート済みのDataFrame
        overall_stats (dict): 全体の統計情報を含む辞書
        source (str): 'robot' または 'aurora'
    """
    if source.lower() == 'robot':
        t_col = 'delta_t_robot_norm'
        R_col = 'delta_R_robot_angle'
        title_prefix = 'Robot'
    elif source.lower() == 'aurora':
        t_col = 'delta_t_aurora_norm'
        R_col = 'delta_R_aurora_angle'
        title_prefix = 'Aurora'
    else:
        print("Error: source must be 'robot' or 'aurora'.")
        return

    # 必要な統計値を取得 (辞書の階層が深くなったため修正)
    try:
        mean_t = overall_stats[t_col]['mean']
        max_t = overall_stats[t_col]['max']
        min_t = overall_stats[t_col]['min']
        
        mean_R = overall_stats[R_col]['mean']
        max_R = overall_stats[R_col]['max']
        min_R = overall_stats[R_col]['min']
        
    except KeyError:
        print(f"Error: Missing mean/max/min values for '{source}' in overall_stats dict.")
        return

    # X軸の設定
    x_labels = df_grouped.index.astype(str)
    x_values = np.arange(len(x_labels)) # 棒グラフの位置

    # 2つのグラフを横に並べて表示
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7)) # 高さを少し増やす
    fig.suptitle(f'{title_prefix} - Motion Delta Analysis (Grouped & Sorted)', fontsize=16)

    # --- グラフ1: Delta Translation Norm ---
    ax1.bar(x_values, df_grouped[t_col], color='royalblue', zorder=3, label='Grouped Mean')
    ax1.set_xlabel('Angle Diff from Base (RotVec[0,0,180]) (deg)', fontsize=12)
    ax1.set_ylabel('Mean Translation Norm (delta_t_norm)', fontsize=12)
    ax1.set_title(f'{title_prefix} - Mean Translation Norm', fontsize=14)
    ax1.set_xticks(x_values)
    ax1.set_xticklabels(x_labels, rotation=70) 
    ax1.grid(axis='y', linestyle='--', alpha=0.7, zorder=0)

    # 全体平均の水平線を追加
    ax1.axhline(y=mean_t, color='red', linestyle='--', linewidth=2, 
                label=f'Overall Mean: {mean_t:.3f}', zorder=4)
    
    # 最大値と最小値をテキストで追加
    stats_text_t = f"Max: {max_t:.3f}\nMin: {min_t:.3f}"
    # グラフの右上にテキストボックスを配置
    ax1.text(0.98, 0.98, stats_text_t, transform=ax1.transAxes, fontsize=10,
             verticalalignment='top', horizontalalignment='right',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
             
    ax1.legend()

    # --- グラフ2: Delta Rotation Angle ---
    ax2.bar(x_values, df_grouped[R_col], color='seagreen', zorder=3, label='Grouped Mean')
    ax2.set_xlabel('Angle Diff from Base (RotVec[0,0,180]) (deg)', fontsize=12)
    ax2.set_ylabel('Mean Rotation Angle (delta_R_angle) (deg)', fontsize=12)
    ax2.set_title(f'{title_prefix} - Mean Rotation Angle', fontsize=14)
    ax2.set_xticks(x_values)
    ax2.set_xticklabels(x_labels, rotation=70)
    ax2.grid(axis='y', linestyle='--', alpha=0.7, zorder=0)

    # 全体平均の水平線を追加
    ax2.axhline(y=mean_R, color='red', linestyle='--', linewidth=2, 
                label=f'Overall Mean: {mean_R:.3f}', zorder=4)

    # 最大値と最小値をテキストで追加
    stats_text_R = f"Max: {max_R:.3f}\nMin: {min_R:.3f}"
    # グラフの右上にテキストボックスを配置
    ax2.text(0.98, 0.98, stats_text_R, transform=ax2.transAxes, fontsize=10,
             verticalalignment='top', horizontalalignment='right',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
             
    ax2.legend()


    plt.tight_layout(rect=[0, 0.03, 1, 0.95]) # メインタイトルと重ならないように調整
    plt.show()


# --- 3. メイン処理（実行） ---
if __name__ == "__main__":
    
    # 1. ファイルパスの指定
    # --- ↓↓↓ ファイルパスをここに入力してください ↓↓↓ ---
    file_path = "robot&aurora/current_code/new_transform/accuracy_test_data/transform_accuracy_orientation_202510290053.csv"
    # --- ↑↑↑ ファイルパスをここに入力してください ↑↑↑ ---
    
    # (または、ターミナルからパスを入力させる場合は以下のコメントを解除)
    # file_path = input("Enter the path to your CSV file: ")

    # 2. データの読み込みと前処理
    #    整数(precision=0)で丸めてグループ化
    grouped_data_df, overall_stats = load_and_group_data(file_path, grouping_precision=0)

    # 3. データ読み込み成功の確認
    if grouped_data_df is None:
        print("Failed to load or process data. Exiting.")
        sys.exit(1) # エラーで終了

    # 4. ユーザーに 'robot' か 'aurora' を選択させる
    choice = ""
    while choice not in ['robot', 'aurora']:
        choice = input("Which data to plot? (Enter 'robot' or 'aurora'): ").lower().strip()
        if choice not in ['robot', 'aurora']:
            print("Invalid input. Please enter 'robot' or 'aurora'.")

    # 5. 選択されたグラフを描画
    print(f"Plotting {choice} data...")
    plot_delta_graphs(grouped_data_df, overall_stats, source=choice)