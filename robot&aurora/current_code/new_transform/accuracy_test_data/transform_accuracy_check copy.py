import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys # sys.exit() を使用するために import

# --- 1. データの準備 ---
# CSVファイルのパスを指定
# !!! この 'your_file_path.csv' を実際のファイルパスに置き換えてください !!!
file_path = 'robot&aurora/current_code/new_transform/accuracy_test_data/transform_accuracy_20251029.csv'


# --- ★ 選択セクション ★ ---
# 'robot' または 'aurora' を選択
target_system = 'aurora'
# target_system = 'aurora'

# ★ フィルタリングの基準を 't' または 'R' で選択 ★
FILTER_TARGET = 'R'
# FILTER_TARGET = 'R' # 'R'でフィルタリングする場合はこちらを有効化

# ★ フィルタリングの閾値 ★
FILTER_THRESHOLD_T = 2.0 # (mm)
FILTER_THRESHOLD_R = 2.0 # (°)

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
    print(f"エラー: 無効な target_system '{target_system}'。'robot' または 'aurora' を選択してください。")
    sys.exit(1)

# 統計表示に使う列のリスト
stat_columns = [filter_column_t, filter_column_r]


# --- 2. データをpandas DataFrameに読み込む ---
try:
    df = pd.read_csv(file_path)
except FileNotFoundError:
    print(f"エラー: ファイルが見つかりません。パスを確認してください: {file_path}")
    sys.exit(1)
except Exception as e:
    print(f"ファイルの読み込み中にエラーが発生しました: {e}")
    sys.exit(1)

# --- ★ 3. データのフィルタリング (選択式) ★ ---
try:
    total_points = len(df)
    
    # 選択に基づいてフィルタリング条件を決定
    if FILTER_TARGET == 't':
        active_filter_col = filter_column_t
        active_threshold = FILTER_THRESHOLD_T
        unit = 'mm'
    elif FILTER_TARGET == 'R':
        active_filter_col = filter_column_r
        active_threshold = FILTER_THRESHOLD_R
        unit = '°'
    else:
        print(f"エラー: 無効な FILTER_TARGET '{FILTER_TARGET}'。't' または 'R' を選択してください。")
        sys.exit(1)

    # フィルタリングを実行
    condition = df[active_filter_col] < active_threshold
    df_filtered = df[condition].copy()
    
    filtered_points = len(df_filtered)
    
    if filtered_points == 0:
        print(f"警告: {active_filter_col} < {active_threshold} {unit} を満たすデータが0件です。")

except KeyError as e:
    print(f"エラー: フィルタリング列 '{e.args[0]}' がCSVファイルに見つかりません。")
    sys.exit(1)
except Exception as e:
    print(f"データのフィルタリング中にエラーが発生しました: {e}")
    sys.exit(1)


# --- 4. 描画する座標データを抽出 (df_filtered を使用) ---
x = df_filtered['robot_z']
y = df_filtered['robot_y']
z = df_filtered['robot_x']

# --- 5. 1x2のサブプロットを作成 ---
fig, axs = plt.subplots(1, 2, figsize=(24, 11), subplot_kw={'projection': '3d'})

axs_flat = axs.flat 

# --- ★ 6. フィルタリング情報のテキストを作成 (選択結果を反映) ★ ---
filter_info_text = (f'Filtered: {filtered_points} / {total_points} points\n'
                    f'(Filter: {FILTER_TARGET} < {active_threshold:.1f} {unit})')
# -------------------------------------------------------------


# --- 7. 2つのグラフをループで描画 ---
for ax, col_name in zip(axs_flat, stat_columns):
    
    # --- 統計情報のテキストを作成 (左上用・各グラフ別) ---
    # (col_name には filter_column_t と filter_column_r が順番に入る)
    try:
        if filtered_points > 0:
            mean_val = df_filtered[col_name].mean()
            max_val = df_filtered[col_name].max()
            min_val = df_filtered[col_name].min()
            
            # ★ 単位を col_name に応じて動的に決定
            stat_unit = 'mm' if col_name == filter_column_t else '°'
            
            stats_text = (f'Stats ({col_name}):\n'
                          f'  Mean: {mean_val:.3f} {stat_unit}\n'
                          f'  Max:  {max_val:.3f} {stat_unit}\n'
                          f'  Min:  {min_val:.3f} {stat_unit}')
        else:
            stats_text = f'Stats ({col_name}):\nN/A'
    except KeyError as e:
        print(f"警告: 統計計算用の列 '{e.args[0]}' が見つかりません。統計表示をスキップします。")
        stats_text = "Stats: N/A (Column not found)"
    # --------------------------------------------------------

    # --- グラフ描画 (単色) ---
    if filtered_points > 0:
        ax.scatter(x, y, z, color=PLOT_COLOR, s=50, marker='o')
    else:
        ax.text2D(0.5, 0.5, "No data to display after filtering", 
                  transform=ax.transAxes, ha='center', va='center', fontsize=14, color='red')

    # --- 視点・軸の設定 ---
    ax.view_init(elev=30, azim=45) 
    ax.invert_yaxis()
    
    # ラベルとタイトルの設定
    ax.set_xlabel('robot_z')
    ax.set_ylabel('robot_y')
    ax.set_zlabel('robot_x (Height)')
    
    # ★ タイトルに選択したフィルタリング条件を明記
    title_text = (f'{plot_title_prefix} - Stats for {col_name}\n'
                  f'(Filtered by: {FILTER_TARGET} < {active_threshold:.1f} {unit})')
    ax.set_title(title_text, fontsize=16, pad=30) 

    # --- 統計情報 (各グラフ別) のテキスト表示 ★ ---
    ax.text2D(0.05, 0.95, stats_text, 
              transform=ax.transAxes, 
              fontsize=12,
              verticalalignment='top', 
              bbox=dict(boxstyle='round,pad=0.3', fc='yellow', alpha=0.5))
    # ----------------------------------------------

    # --- フィルタリング情報のテキスト表示 (右上・共通) ★ ---
    ax.text2D(0.95, 0.95, filter_info_text, 
              transform=ax.transAxes, 
              fontsize=12,
              verticalalignment='top',   
              horizontalalignment='right', 
              bbox=dict(boxstyle='round,pad=0.3', fc='cyan', alpha=0.5))
    # ------------------------------------------

# --- 8. レイアウトの調整と表示 ---
plt.subplots_adjust(wspace=0.2, hspace=0.3, left=0.05, right=0.95, top=0.9, bottom=0.1)

# グラフを画面に表示
plt.show()