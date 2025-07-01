import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Font settings for English display
plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['axes.unicode_minus'] = False

# CSVファイルパスを設定
csv_file_path = r"robot&aurora\current_code\offset_test_data\aurora_robot_sequential_rotation_log_y148.5.csv"

# CSVファイルからデータを読み込み
try:
    df = pd.read_csv(csv_file_path)
    print(f"Data loaded successfully: {len(df)} points")
except FileNotFoundError:
    print(f"Error: File not found {csv_file_path}")
    exit()
except Exception as e:
    print(f"Error: {e}")
    exit()

print("\n" + "="*50)
print("Aurora Coordinate Scatter Analysis (Basic Statistics)")
print("="*50)

# Group data by axis type
axis_groups = df.groupby('axis')
print(f"Analysis axes: {list(axis_groups.groups.keys())}")

# Store basic statistics
basic_stats = {}

for axis_name, group in axis_groups:
    ax_x = group['aurora_x'].values
    ax_y = group['aurora_y'].values  
    ax_z = group['aurora_z'].values
    
    stats = {
        'count': len(group),
        'x_mean': np.mean(ax_x),
        'y_mean': np.mean(ax_y),
        'z_mean': np.mean(ax_z),
        'x_std': np.std(ax_x, ddof=1),
        'y_std': np.std(ax_y, ddof=1),
        'z_std': np.std(ax_z, ddof=1),
        'x_var': np.var(ax_x, ddof=1),
        'y_var': np.var(ax_y, ddof=1),
        'z_var': np.var(ax_z, ddof=1),
        'x_range': ax_x.max() - ax_x.min(),
        'y_range': ax_y.max() - ax_y.min(),
        'z_range': ax_z.max() - ax_z.min(),
        'x_min': ax_x.min(),
        'x_max': ax_x.max(),
        'y_min': ax_y.min(),
        'y_max': ax_y.max(),
        'z_min': ax_z.min(),
        'z_max': ax_z.max()
    }
    
    basic_stats[axis_name] = stats
    
    print(f"\n[{axis_name.upper()} AXIS] ({stats['count']} points)")
    print(f"  Mean: X={stats['x_mean']:.2f}, Y={stats['y_mean']:.2f}, Z={stats['z_mean']:.2f}")
    print(f"  Std Dev: X={stats['x_std']:.3f}, Y={stats['y_std']:.3f}, Z={stats['z_std']:.3f}")
    print(f"  Variance: X={stats['x_var']:.3f}, Y={stats['y_var']:.3f}, Z={stats['z_var']:.3f}")
    print(f"  Range: X={stats['x_range']:.2f}, Y={stats['y_range']:.2f}, Z={stats['z_range']:.2f}")
    print(f"  X coordinate range: {stats['x_min']:.2f} < aurora_x < {stats['x_max']:.2f}")
    print(f"  Y coordinate range: {stats['y_min']:.2f} < aurora_y < {stats['y_max']:.2f}")
    print(f"  Z coordinate range: {stats['z_min']:.2f} < aurora_z < {stats['z_max']:.2f}")

# Overall statistics
all_x = df['aurora_x'].values
all_y = df['aurora_y'].values
all_z = df['aurora_z'].values

overall_stats = {
    'x_mean': np.mean(all_x),
    'y_mean': np.mean(all_y),
    'z_mean': np.mean(all_z),
    'x_std': np.std(all_x, ddof=1),
    'y_std': np.std(all_y, ddof=1),
    'z_std': np.std(all_z, ddof=1),
    'x_var': np.var(all_x, ddof=1),
    'y_var': np.var(all_y, ddof=1),
    'z_var': np.var(all_z, ddof=1),
    'x_min': all_x.min(),
    'x_max': all_x.max(),
    'y_min': all_y.min(),
    'y_max': all_y.max(),
    'z_min': all_z.min(),
    'z_max': all_z.max()
}

print(f"\n[OVERALL STATISTICS]")
print(f"  Mean: X={overall_stats['x_mean']:.2f}, Y={overall_stats['y_mean']:.2f}, Z={overall_stats['z_mean']:.2f}")
print(f"  Std Dev: X={overall_stats['x_std']:.3f}, Y={overall_stats['y_std']:.3f}, Z={overall_stats['z_std']:.3f}")
print(f"  Variance: X={overall_stats['x_var']:.3f}, Y={overall_stats['y_var']:.3f}, Z={overall_stats['z_var']:.3f}")
print(f"  X coordinate range: {overall_stats['x_min']:.2f} < aurora_x < {overall_stats['x_max']:.2f}")
print(f"  Y coordinate range: {overall_stats['y_min']:.2f} < aurora_y < {overall_stats['y_max']:.2f}")
print(f"  Z coordinate range: {overall_stats['z_min']:.2f} < aurora_z < {overall_stats['z_max']:.2f}")

# Standard deviation scatter comparison
print(f"\n{'='*30}")
print("Scatter Comparison by Standard Deviation")
print("="*30)

axes = list(basic_stats.keys())
for coord in ['x', 'y', 'z']:
    print(f"\n{coord.upper()} coordinate standard deviation:")
    sorted_axes = sorted(axes, key=lambda a: basic_stats[a][f'{coord}_std'], reverse=True)
    for i, axis in enumerate(sorted_axes, 1):
        std_val = basic_stats[axis][f'{coord}_std']
        min_val = basic_stats[axis][f'{coord}_min']
        max_val = basic_stats[axis][f'{coord}_max']
        print(f"  {i}. {axis.upper()} axis: {std_val:.3f} (range: {min_val:.2f} < aurora_{coord} < {max_val:.2f})")

# Visualization (graphs + statistics display)
fig = plt.figure(figsize=(24, 12))

# 1. 3D scatter plot
colors = {'roll': 'red', 'pitch': 'green', 'yaw': 'blue'}
ax1 = fig.add_subplot(2, 3, 1, projection='3d')

for axis_name, group in axis_groups:
    ax_x = group['aurora_x'].values
    ax_y = group['aurora_y'].values
    ax_z = group['aurora_z'].values
    ax1.scatter(ax_x, ax_y, ax_z, c=colors[axis_name], 
                label=f'{axis_name.upper()} axis', s=50, alpha=0.7)

ax1.set_xlabel('Aurora X')
ax1.set_ylabel('Aurora Y')
ax1.set_zlabel('Aurora Z')
ax1.set_title('Aurora Coordinate Distribution')
ax1.legend()
ax1.grid(True)

# 2. Standard deviation comparison
ax2 = fig.add_subplot(2, 3, 2)
x_stds = [basic_stats[axis]['x_std'] for axis in axes]
y_stds = [basic_stats[axis]['y_std'] for axis in axes]
z_stds = [basic_stats[axis]['z_std'] for axis in axes]

x = np.arange(len(axes))
width = 0.25

ax2.bar(x - width, x_stds, width, label='X coordinate', alpha=0.8)
ax2.bar(x, y_stds, width, label='Y coordinate', alpha=0.8)
ax2.bar(x + width, z_stds, width, label='Z coordinate', alpha=0.8)

ax2.set_xlabel('Rotation Axis')
ax2.set_ylabel('Standard Deviation')
ax2.set_title('Standard Deviation Comparison')
ax2.set_xticks(x)
ax2.set_xticklabels([axis.upper() for axis in axes])
ax2.legend()
ax2.grid(True, alpha=0.3)

# 3. Variance comparison
ax3 = fig.add_subplot(2, 3, 3)
x_vars = [basic_stats[axis]['x_var'] for axis in axes]
y_vars = [basic_stats[axis]['y_var'] for axis in axes]
z_vars = [basic_stats[axis]['z_var'] for axis in axes]

ax3.bar(x - width, x_vars, width, label='X coordinate', alpha=0.8)
ax3.bar(x, y_vars, width, label='Y coordinate', alpha=0.8)
ax3.bar(x + width, z_vars, width, label='Z coordinate', alpha=0.8)

ax3.set_xlabel('Rotation Axis')
ax3.set_ylabel('Variance')
ax3.set_title('Variance Comparison')
ax3.set_xticks(x)
ax3.set_xticklabels([axis.upper() for axis in axes])
ax3.legend()
ax3.grid(True, alpha=0.3)

# 4. Statistics display area 1 (coordinate ranges by axis)
ax4 = fig.add_subplot(2, 3, 4)
ax4.axis('off')  # Hide axes

# Create statistics text
stats_text = "=== Coordinate Ranges by Axis ===\n\n"
for axis_name in axes:
    stats = basic_stats[axis_name]
    stats_text += f"[{axis_name.upper()} AXIS]\n"
    stats_text += f"{stats['x_min']:.2f} < aurora_x < {stats['x_max']:.2f}\n"
    stats_text += f"{stats['y_min']:.2f} < aurora_y < {stats['y_max']:.2f}\n"
    stats_text += f"{stats['z_min']:.2f} < aurora_z < {stats['z_max']:.2f}\n\n"

ax4.text(0.05, 0.95, stats_text, transform=ax4.transAxes, fontsize=12,
         verticalalignment='top', fontfamily='monospace',
         bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgray", alpha=0.8))

# 5. Statistics display area 2 (overall coordinate ranges)
ax5 = fig.add_subplot(2, 3, 5)
ax5.axis('off')  # Hide axes

overall_text = "=== Overall Coordinate Ranges ===\n\n"
overall_text += f"[ALL DATA]\n"
overall_text += f"{overall_stats['x_min']:.2f} < aurora_x < {overall_stats['x_max']:.2f}\n"
overall_text += f"{overall_stats['y_min']:.2f} < aurora_y < {overall_stats['y_max']:.2f}\n"
overall_text += f"{overall_stats['z_min']:.2f} < aurora_z < {overall_stats['z_max']:.2f}\n"

ax5.text(0.05, 0.95, overall_text, transform=ax5.transAxes, fontsize=12,
         verticalalignment='top', fontfamily='monospace',
         bbox=dict(boxstyle="round,pad=0.5", facecolor="lightyellow", alpha=0.8))

# 6. Empty area (for other information display if needed)
ax6 = fig.add_subplot(2, 3, 6)
ax6.axis('off')  # Hide axes

plt.tight_layout()
plt.show()

# Conclusion
print(f"\n{'='*40}")
print("Analysis Results")
print("="*40)

# Identify the rotation axis with the largest scatter for each coordinate axis
for coord in ['X', 'Y', 'Z']:
    coord_lower = coord.lower()
    most_scattered = max(axes, key=lambda a: basic_stats[a][f'{coord_lower}_std'])
    max_std = basic_stats[most_scattered][f'{coord_lower}_std']
    print(f"{coord} coordinate largest scatter: {most_scattered.upper()} axis (std dev: {max_std:.3f})")

# Overall scatter (average of standard deviations for each axis)
overall_scatter = {}
for axis in axes:
    avg_std = (basic_stats[axis]['x_std'] + basic_stats[axis]['y_std'] + basic_stats[axis]['z_std']) / 3
    overall_scatter[axis] = avg_std

most_scattered_overall = max(overall_scatter.keys(), key=lambda a: overall_scatter[a])
print(f"\nOverall largest scatter: {most_scattered_overall.upper()} axis")
print(f"(average std dev: {overall_scatter[most_scattered_overall]:.3f})")

for axis in sorted(overall_scatter.keys(), key=lambda a: overall_scatter[a], reverse=True):
    print(f"  {axis.upper()} axis: {overall_scatter[axis]:.3f}")