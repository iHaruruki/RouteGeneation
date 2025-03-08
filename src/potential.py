import numpy as np
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ここに地図のファイルパスを指定
MAP_FILE = "..\maps\map3.pgm"

# 地図をロード
def load_map(file_path, threshold=220):
    img = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"画像ファイルが読み込めませんでした: {file_path}")
    _, binary = cv2.threshold(img, threshold, 255, cv2.THRESH_BINARY)
    occupancy_grid = (binary == 0).astype(np.uint8)
    return occupancy_grid

# ポテンシャル法
def potential_field(grid, start, goal, max_iterations=5000, attraction_coeff=5.0, repulsion_coeff=50.0, repulsion_radius=5):
    grid_size = grid.shape
    potential = np.zeros(grid_size)

    # ゴールの引力場を設定
    y_indices, x_indices = np.indices(grid_size)
    potential += attraction_coeff * np.hypot(goal[0]-y_indices, goal[1]-x_indices)

    # 障害物の斥力場を追加
    obstacle_y, obstacle_x = np.where(grid == 1)
    for oy, ox in zip(obstacle_y, obstacle_x):
        for y in range(max(0, oy - repulsion_radius), min(grid_size[0], oy + repulsion_radius)):
            for x in range(max(0, ox - repulsion_radius), min(grid_size[1], ox + repulsion_radius)):
                distance = np.hypot(oy-y, ox-x)
                if distance == 0:
                    continue
                potential[y, x] += repulsion_coeff / distance

    path = [start]
    current = start
    for _ in range(max_iterations):
        if current == goal:
            break
        min_potential = float('inf')
        next_node = None
        for dy, dx in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            ny, nx = current[0]+dy, current[1]+dx
            if 0 <= ny < grid_size[0] and 0 <= nx < grid_size[1] and grid[ny, nx] == 0:
                if potential[ny, nx] < min_potential:
                    min_potential = potential[ny, nx]
                    next_node = (ny, nx)
        if next_node is None or next_node == current:
            return [], potential
        current = next_node
        path.append(current)

    return path, potential

# 経路を地図上に表示
def plot_path(grid, path, start, goal):
    plt.imshow(grid, cmap='gray_r')
    path_y, path_x = zip(*path)
    plt.plot(path_x, path_y, color="blue", linewidth=2, label="Path")
    plt.scatter([start[1]], [start[0]], marker="o", color="green", s=100, label="Start")
    plt.scatter([goal[1]], [goal[0]], marker="x", color="red", s=100, label="Goal")
    plt.legend()
    plt.title("Path Planning")
    plt.show()

# 地図確認用表示
def plot_start_goal(grid, start, goal):
    plt.imshow(grid, cmap='gray_r')
    plt.scatter([start[1]], [start[0]], marker="o", color="green", s=100, label="Start")
    plt.scatter([goal[1]], [goal[0]], marker="x", color="red", s=100, label="Goal")
    plt.legend()
    plt.title("Start and Goal position check")
    plt.show()

# ポテンシャル場の3次元表示
def plot_potential_field_3d(potential):
    X, Y = np.meshgrid(np.arange(potential.shape[1]), np.arange(potential.shape[0]))
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(X, Y, potential, cmap='jet')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Potential')
    ax.set_title("3D Potential Field")
    plt.show()

# メイン関数
if __name__ == "__main__":
    grid = load_map(MAP_FILE)

    # スタートとゴールの座標を設定 (y, x)の形式で指定
    start = (20, 20)
    goal = (90, 90)

    plot_start_goal(grid, start, goal)  # 位置確認用表示

    path, potential = potential_field(grid, start, goal)

    plot_potential_field_3d(potential)  # ポテンシャル場を3次元で表示

    if path and path[-1] == goal:
        print(f"経路が見つかりました！ステップ数: {len(path)}")
        plot_path(grid, path, start, goal)
    else:
        print("経路が見つかりませんでした。")
