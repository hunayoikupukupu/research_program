import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys # sys.exit() を使用するために import

# --- 1. データの準備 ---
# CSVファイルのパスを指定
# !!! この 'your_file_path.csv' を実際のファイルパスに置き換えてください !!!
file_path = 'robot&aurora/current_code/new_transform/accuracy_test_data/transform_accuracy_20251029.csv'


# --- ★ 選択セクション ★ ---
# ここで 'robot' または 'aurora' を選択してください
target_system = 'robot'
# target_system = 'aurora' # こちらを有効にする場合は、上の行をコメントアウト

# 選択に基づいて色付けに使用する列名を決定
if target_system == 'robot':
    color_columns = ['delta_t_robot_norm', 'delta_R_robot_angle']
    plot_title_prefix = 'Robot'
elif target_system == 'aurora':
    color_columns = ['delta_t_aurora_norm', 'delta_R_aurora_angle']
    plot_title_prefix = 'Aurora'
else:
    print(f"エラー: 無効な target_system '{target_system}'。'robot' または 'aurora' を選択してください。")
    sys.exit(1) # スクリプトを停止


# --- 2. データをpandas DataFrameに読み込む ---
try:
    df = pd.read_csv(file_path)
except FileNotFoundError:
    print(f"エラー: ファイルが見つかりません。パスを確認してください: {file_path}")
    sys.exit(1) # スクリプトを停止
except Exception as e:
    print(f"ファイルの読み込み中にエラーが発生しました: {e}")
    sys.exit(1) # スクリプトを停止

# --- 3. 描画する座標データを抽出 (前回と同じ) ---
# X軸に 'robot_z' を割り当て
x = df['robot_z']
# Y軸に 'robot_y' を割り当て
y = df['robot_y']
# Z軸 (高さ) に 'robot_x' を割り当て
z = df['robot_x']

# --- 4. 1x2のサブプロットを作成 (グラフ2つに変更) ---
# figsizeを横長に変更し、グラフ2つに対応
fig, axs = plt.subplots(1, 2, figsize=(24, 11), subplot_kw={'projection': '3d'})

# axsが1次元配列になるため、axs[i]でアクセス (または .flat を使う)
if len(color_columns) == 1:
    axs_flat = [axs] # グラフが1つの場合
else:
    axs_flat = axs.flat # グラフが複数の場合 (今回は2つ)

# --- 5. 2つのグラフをループで描画 ---
for ax, col_name in zip(axs_flat, color_columns):
    
    try:
        colors = df[col_name]
    except KeyError:
        print(f"警告: 列 '{col_name}' が見つかりません。このグラフをスキップします。")
        continue

    mean_val = colors.mean()

    scatter = ax.scatter(x, y, z, c=colors, cmap='jet', s=50, marker='o')

    # --- 視点の変更 ★ ---
    # 仰角(elev)を30度、方位角(azim)を45度に変更
    ax.view_init(elev=30, azim=45) 

    # ラベルとタイトルの設定
    ax.set_xlabel('robot_z')
    ax.set_ylabel('robot_y')
    ax.set_zlabel('robot_x (Height)')
    # タイトルに選択したシステム名('Robot' or 'Aurora')を追加
    ax.set_title(f'{plot_title_prefix} - Color: {col_name}', fontsize=16, pad=30) 

    # カラーバーの設定 (padを少し調整)
    cbar = fig.colorbar(scatter, ax=ax, shrink=0.6, aspect=15, pad=0.1) 
    cbar.set_label(col_name, fontsize=12)

    # 平均値のテキスト表示
    ax.text2D(0.05, 0.95, f'Mean: {mean_val:.3f}', 
              transform=ax.transAxes, 
              fontsize=12,
              bbox=dict(boxstyle='round,pad=0.3', fc='yellow', alpha=0.5))

# --- 6. レイアウトの調整と表示 ---
# wspaceを調整 (左右のグラフの間隔)
plt.subplots_adjust(wspace=0.2, hspace=0.3, left=0.05, right=0.95, top=0.9, bottom=0.1)

# グラフを画面に表示
plt.show()