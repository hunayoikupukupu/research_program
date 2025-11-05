import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import os
import sys

# --- 1. データの読み込みと前処理 (★修正: データ数を表示) ---

def load_and_group_data(filepath, bin_width=3, bin_start=None):
    """
    CSVを読み込み、angle_diffを計算し、指定された範囲(bin_width)と
    開始位置(bin_start)でグループ化された平均値と、
    全体の統計情報（平均、最大、最小）の両方を返します。
    (★各グループのデータ数をコンソールに表示する機能を追加)

    Args:
        filepath (str): CSVファイルへのパス
        bin_width (float or int): angle_diffをグループ化する際の範囲の幅 (例: 3)
        bin_start (float or int, optional): 
            ビニングを開始する最小値 (例: 5)。
            Noneの場合はデータの最小値から開始。

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

    # --- 6. 【グループ化】 ---
    if df['angle_diff'].empty:
        print("Error: 'angle_diff' column is empty or calculation failed.")
        return None, None
        
    try:
        # ビンの最大値はデータ全体をカバーするように動的に決定
        max_val = np.ceil(df['angle_diff'].max())
        
        # bin_start が指定されているか確認
        if bin_start is not None:
            min_val = bin_start
            print(f"Grouping by 'angle_diff' starting from {bin_start} with width {bin_width}...")
        else:
            # 指定がない場合は従来通り、データの最小値から
            min_val = np.floor(df['angle_diff'].min())
            print(f"Grouping by 'angle_diff' from min value ({min_val}) with width {bin_width}...")

        # np.arange を使い、min_val から max_val を超えるまで bin_width 刻みでビンを作成
        bins = np.arange(min_val, max_val + bin_width, bin_width)
        
        if len(bins) < 2:
            # データが開始値より小さい場合など
            bins = np.array([min_val, min_val + bin_width])
            print(f"Warning: Data max ({max_val}) might be less than bin_start ({min_val}). Creating first bin ({bins[0]}, {bins[1]}] anyway.")

        print(f"Bins created: {bins}")
        
        # pd.cutでグループ化
        df['angle_diff_group'] = pd.cut(df['angle_diff'], bins=bins, right=True)

    except Exception as e:
        print(f"Error during binning (pd.cut): {e}")
        return None, None

    # --- ▼ (★修正箇所) 各グループのデータ数を計算して表示 ▼ ---
    print("\n--- Data Counts per Group ---")
    # `dropna=False` で NaN (ビン外) の数も表示
    # `sort_index()` でビンの昇順に並び替え
    group_counts = df['angle_diff_group'].value_counts(dropna=False).sort_index()
    print(group_counts)
    print("-------------------------------\n")
    # --- ▲ (修正箇所 完了) ▲ ---


    # 7. グループ化して平均を計算
    print("Calculating mean for each group...")
    cols_to_average = [
        'delta_t_robot_norm', 'delta_R_robot_angle',
        'delta_t_aurora_norm', 'delta_R_aurora_angle'
    ]
    
    df_grouped = df.groupby('angle_diff_group')[cols_to_average].mean()

    # 8. 【ソート】
    df_grouped = df_grouped.sort_index(ascending=True)
    
    # 9. (追加) プロットしやすいようにインデックスを文字列に変換
    df_grouped.index = df_grouped.index.astype(str)

    print("Preprocessing and grouping complete.")
    return df_grouped, overall_stats


# --- 2. グラフの描画 (変更なし) ---

def plot_delta_graphs(df_grouped, overall_stats, source='robot'):
    """
    グループ化されたDataFrameに基づいて、棒グラフと全体の平均線、
    および最大/最小値をテキストで描画します。
    (X軸はビニングされたカテゴリ)
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

    # 必要な統計値を取得
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

    # --- X軸の設定 ---
    x_labels = df_grouped.index
    x_values = np.arange(len(x_labels)) # 棒グラフの位置 [0, 1, 2, ...]
    
    if len(x_values) == 0:
        print("Error: No data to plot after grouping.")
        return
    
    # --- X軸の目盛り間引き処理 ---
    num_ticks = len(x_values)
    
    if num_ticks > 30: # 閾値（例: 30）
        step = max(1, num_ticks // 15) # 15個程度のラベルが表示されるように調整
        tick_indices = np.arange(0, num_ticks, step)
        
        if tick_indices[-1] != num_ticks - 1:
             tick_indices = np.append(tick_indices, num_ticks - 1)
        
        tick_indices = np.unique(tick_indices)

        ticks_to_show = x_values[tick_indices]
        labels_to_show = x_labels[tick_indices]
    else:
        # ラベルが少ない場合は全て表示
        ticks_to_show = x_values
        labels_to_show = x_labels


    # 2つのグラフを横に並べて表示
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))
    fig.suptitle(f'{title_prefix} - Motion Delta Analysis (Grouped by Angle Range)', fontsize=16)

    # --- グラフ1: Delta Translation Norm ---
    
    ax1.bar(x_values, df_grouped[t_col], color='royalblue', zorder=3, label='Grouped Mean', width=0.9) 
    
    ax1.set_xlabel('Angle Diff from Base (RotVec[0,0,180]) (deg)', fontsize=12)
    ax1.set_ylabel('Mean Translation Norm (delta_t_norm)', fontsize=12)
    ax1.set_title(f'{title_prefix} - Mean Translation Norm', fontsize=14)
    ax1.grid(axis='y', linestyle='--', alpha=0.7, zorder=0)

    # X軸の目盛りとラベルを設定
    ax1.set_xlim(-0.5, len(x_values) - 0.5)
    ax1.set_xticks(ticks_to_show)
    ax1.set_xticklabels(labels_to_show, rotation=70) 

    # 全体平均の水平線
    ax1.axhline(y=mean_t, color='red', linestyle='--', linewidth=2, 
                  label=f'Overall Mean: {mean_t:.3f}', zorder=4)
    
    # 最大値と最小値のテキスト
    stats_text_t = f"Max: {max_t:.3f}\nMin: {min_t:.3f}"
    ax1.text(0.98, 0.98, stats_text_t, transform=ax1.transAxes, fontsize=10,
             verticalalignment='top', horizontalalignment='right',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
             
    ax1.legend()

    # --- グラフ2: Delta Rotation Angle ---
    
    ax2.bar(x_values, df_grouped[R_col], color='seagreen', zorder=3, label='Grouped Mean', width=0.9)
    
    ax2.set_xlabel('Angle Diff from Base (RotVec[0,0,180]) (deg)', fontsize=12)
    ax2.set_ylabel('Mean Rotation Angle (delta_R_angle) (deg)', fontsize=12)
    ax2.set_title(f'{title_prefix} - Mean Rotation Angle', fontsize=14)
    ax2.grid(axis='y', linestyle='--', alpha=0.7, zorder=0)

    # X軸の目盛りとラベルを設定 (ax1と同様)
    ax2.set_xlim(-0.5, len(x_values) - 0.5)
    ax2.set_xticks(ticks_to_show)
    ax2.set_xticklabels(labels_to_show, rotation=70)

    # 全体平均の水平線
    ax2.axhline(y=mean_R, color='red', linestyle='--', linewidth=2, 
                  label=f'Overall Mean: {mean_R:.3f}', zorder=4)

    # 最大値と最小値のテキスト
    stats_text_R = f"Max: {max_R:.3f}\nMin: {min_R:.3f}"
    ax2.text(0.98, 0.98, stats_text_R, transform=ax2.transAxes, fontsize=10,
             verticalalignment='top', horizontalalignment='right',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
             
    ax2.legend()

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    # グラフを表示します
    plt.show()


# --- 3. メイン処理（実行） (変更なし) ---
if __name__ == "__main__":
    
    # 1. ファイルパスの指定
    # --- ↓↓↓ ★★★【重要】★★★ ↓↓↓ ---
    # 以下のパスを、あなたの環境のCSVファイルの
    # 正しいパス (例: "C:/Users/YourUser/Desktop/data.csv" や "/home/user/my_data.csv")
    # に書き換えてください。
    
    file_path = "robot&aurora/current_code/new_transform/accuracy_test_data/transform_accuracy_orientation_202510290053.csv"
    
    # --- ↑↑↑ ★★★【重要】★★★ ↑↑↑ ---
    
    
    # (または、ターミナルからパスを入力させる場合は以下のコメントを解除)
    # file_path = input("Enter the path to your CSV file: ")
    
    # (★ご要望) 3度の範囲で、5から開始
    bin_width_degrees = 4 
    bin_start_value = 6

    # 2. データの読み込みと前処理
    grouped_data_df, overall_stats = load_and_group_data(
        file_path, 
        bin_width=bin_width_degrees, 
        bin_start=bin_start_value
    )

    # 3. データ読み込み成功の確認
    if grouped_data_df is None:
        print("Failed to load or process data. Exiting.")
        sys.exit(1)
        
    if grouped_data_df.empty:
        print("No data was grouped (e.g., all data fell outside bins or was NaN). Exiting.")
        sys.exit(1)

    # 4. ユーザーに 'robot' か 'aurora' を選択させる
    choice = ""
    while choice not in ['robot', 'aurora']:
        choice = input("Which data to plot? (Enter 'robot' or 'aurora'): ").lower().strip()
        if choice not in ['robot', 'aurora']:
            print("Invalid input. Please enter 'robot' or 'aurora'.")

    # 5. 選択されたグラフを描画
    print(f"Plotting {choice} data...")
    plot_delta_graphs(grouped_data_df, overall_stats, source=choice)