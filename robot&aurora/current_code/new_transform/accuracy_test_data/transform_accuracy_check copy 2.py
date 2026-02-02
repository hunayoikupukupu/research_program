import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys

# --- 1. データの準備 ---
file_path = 'robot&aurora/current_code/new_transform/accuracy_test_data/transform_accuracy_20251029.csv'

# --- ★ 選択セクション ★ ---
target_system = 'aurora'  # 'robot' または 'aurora'

# ★ フィルタリングの基準を選択 ★
# 't' : 変換誤差(mm)のみ
# 'R' : 回転誤差(°)のみ
# 'both' : t と R の両方を満たす (AND条件)
FILTER_TARGET = 'both' 

# ★ フィルタリングの閾値 ★
FILTER_THRESHOLD_T = 2.0  # (mm)
FILTER_THRESHOLD_R = 2.0  # (°)

# ★ 点の描画色 ★
PLOT_COLOR = 'blue'
# -----------------------------

# 選択に基づいて使用する列名を決定
if target_system == 'robot':
    plot_title_prefix = 'Robot'
    filter_column_t = 'delta_t_robot_norm'  
    filter_column_r = 'delta_R_robot_angle' 
elif target_system == 'aurora':
    plot_title_prefix = 'Aurora'
    filter_column_t = 'delta_t_aurora_norm' 
    filter_column_r = 'delta_R_aurora_angle'
else:
    print(f"エラー: 無効な target_system '{target_system}'。")
    sys.exit(1)

stat_columns = [filter_column_t, filter_column_r]

# --- 2. データを読み込む ---
try:
    df = pd.read_csv(file_path)
except Exception as e:
    print(f"読み込みエラー: {e}")
    sys.exit(1)

# --- ★ 3. データのフィルタリング (AND条件対応) ★ ---
try:
    total_points = len(df)
    
    # フィルタ条件の構築
    cond_t = df[filter_column_t] < FILTER_THRESHOLD_T
    cond_r = df[filter_column_r] < FILTER_THRESHOLD_R

    if FILTER_TARGET == 't':
        condition = cond_t
        filter_desc = f"{FILTER_TARGET} < {FILTER_THRESHOLD_T} mm"
    elif FILTER_TARGET == 'R':
        condition = cond_r
        filter_desc = f"{FILTER_TARGET} < {FILTER_THRESHOLD_R} °"
    elif FILTER_TARGET == 'both':
        # AND条件: 両方の条件を満たす行のみ抽出
        condition = cond_t & cond_r
        filter_desc = f"t<{FILTER_THRESHOLD_T}mm AND R<{FILTER_THRESHOLD_R}°"
    else:
        print(f"エラー: 無効な FILTER_TARGET '{FILTER_TARGET}'。")
        sys.exit(1)

    df_filtered = df[condition].copy()
    filtered_points = len(df_filtered)
    
    if filtered_points == 0:
        print(f"警告: 条件を満たすデータが0件です。")

except KeyError as e:
    print(f"エラー: 列 '{e.args[0]}' がCSVに見つかりません。")
    sys.exit(1)

# --- 4. 座標抽出 ---
x = df_filtered['robot_z']
y = df_filtered['robot_y']
z = df_filtered['robot_x']

# --- 5. サブプロット作成 ---
fig, axs = plt.subplots(1, 2, figsize=(24, 11), subplot_kw={'projection': '3d'})
axs_flat = axs.flat 

# --- 6. フィルタリング情報のテキスト ---
filter_info_text = (f'Filtered: {filtered_points} / {total_points} points\n'
                    f'Filter: {filter_desc}')

# --- 7. 描画ループ ---
for ax, col_name in zip(axs_flat, stat_columns):
    # 各列の統計
    if filtered_points > 0:
        mean_val = df_filtered[col_name].mean()
        max_val = df_filtered[col_name].max()
        min_val = df_filtered[col_name].min()
        stat_unit = 'mm' if col_name == filter_column_t else '°'
        stats_text = (f'Stats ({col_name}):\n'
                      f'  Mean: {mean_val:.3f} {stat_unit}\n'
                      f'  Max:  {max_val:.3f} {stat_unit}\n'
                      f'  Min:  {min_val:.3f} {stat_unit}')
    else:
        stats_text = f'Stats ({col_name}):\nN/A'

    # 散布図描画
    if filtered_points > 0:
        ax.scatter(x, y, z, color=PLOT_COLOR, s=50, marker='o')
    else:
        ax.text2D(0.5, 0.5, "No data to display", transform=ax.transAxes, ha='center', color='red')

    # 設定
    ax.view_init(elev=30, azim=45) 
    ax.invert_yaxis()
    ax.set_xlabel('robot_z')
    ax.set_ylabel('robot_y')
    ax.set_zlabel('robot_x (Height)')
    
    title_text = f'{plot_title_prefix} - Stats for {col_name}\n({filter_desc})'
    ax.set_title(title_text, fontsize=16, pad=30) 

    # 左上：統計情報
    ax.text2D(0.05, 0.95, stats_text, transform=ax.transAxes, fontsize=12,
              verticalalignment='top', bbox=dict(boxstyle='round,pad=0.3', fc='yellow', alpha=0.5))

    # 右上：全体フィルタ情報
    ax.text2D(0.95, 0.95, filter_info_text, transform=ax.transAxes, fontsize=12,
              verticalalignment='top', horizontalalignment='right', 
              bbox=dict(boxstyle='round,pad=0.3', fc='cyan', alpha=0.5))

plt.subplots_adjust(wspace=0.2, hspace=0.3, left=0.05, right=0.95, top=0.9, bottom=0.1)
plt.show()