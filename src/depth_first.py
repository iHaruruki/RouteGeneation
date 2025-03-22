import numpy as np
import cv2
import matplotlib.pyplot as plt

# ここに地図のファイルパスを指定
MAP_FILE = "..\maps\maze_map.pgm"

# 地図をロード
def load_map(file_path, threshold=220):
    img = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"画像ファイルが読み込めませんでした: {file_path}")
    _, binary = cv2.threshold(img, threshold, 255, cv2.THRESH_BINARY)
    occupancy_grid = (binary == 0).astype(np.uint8)
    return occupancy_grid

# 深さ優先探索 (DFS)
def dfs(grid, start, goal):
    neighbors = [(0,1), (1,0), (-1,0), (0,-1),
                 (1,1), (-1,1), (1,-1), (-1,-1)]
    stack = [start]
    came_from = {start: None}

    while stack:
        current = stack.pop()
        if current == goal:
            break

        for dx, dy in neighbors:
            next_node = (current[0]+dx, current[1]+dy)
            if (0 <= next_node[0] < grid.shape[0] and
                0 <= next_node[1] < grid.shape[1] and
                grid[next_node] == 0 and
                next_node not in came_from):
                stack.append(next_node)
                came_from[next_node] = current

    # 経路復元
    if goal not in came_from:
        return []
    path = []
    current = goal
    while current:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path

# 経路を地図上に表示
def plot_path(grid, path, start, goal):
    plt.imshow(grid, cmap='gray_r')
    path_y, path_x = zip(*path)
    plt.plot(path_x, path_y, color="blue", linewidth=2, label="Path")
    plt.scatter([start[1]], [start[0]], marker="o", color="green", s=100, label="Start")
    plt.scatter([goal[1]], [goal[0]], marker="x", color="red", s=100, label="Goal")
    plt.legend()
    plt.show()

# 地図確認用表示
def plot_start_goal(grid, start, goal):
    plt.imshow(grid, cmap='gray_r')
    plt.scatter([start[1]], [start[0]], marker="o", color="green", s=100, label="Start")
    plt.scatter([goal[1]], [goal[0]], marker="x", color="red", s=100, label="Goal")
    plt.legend()
    plt.title("Start and Goal position check")
    plt.show()

# メイン関数
if __name__ == "__main__":
    grid = load_map(MAP_FILE)

    # スタートとゴールの座標を設定 (y, x)の形式で指定
    start = (30, 30)
    goal = (50, 400)

    plot_start_goal(grid, start, goal)  # 位置確認用表示

    path = dfs(grid, start, goal)

    if path:
        print(f"経路が見つかりました！ステップ数: {len(path)}")
        plot_path(grid, path, start, goal)
    else:
        print("経路が見つかりませんでした。")
