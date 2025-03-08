import numpy as np
import cv2

# 迷路のサイズ（ピクセル単位）
width, height = 400, 400
cell_size = 20  # 迷路のセルのサイズ

# 迷路を格納する配列（1 = 壁, 0 = 通路）
maze = np.ones((height // cell_size, width // cell_size), dtype=np.uint8)

# 深さ優先探索で迷路を作成
def generate_maze(x, y):
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    np.random.shuffle(directions)

    for dx, dy in directions:
        nx, ny = x + dx * 2, y + dy * 2
        if 0 <= nx < maze.shape[1] and 0 <= ny < maze.shape[0] and maze[ny, nx] == 1:
            maze[y + dy, x + dx] = 0  # 道を開ける
            maze[ny, nx] = 0
            generate_maze(nx, ny)

# 迷路の初期位置
maze[1, 1] = 0
generate_maze(1, 1)

# 画像に変換（白＝自由空間, 黒＝壁）
img = np.ones((height, width), dtype=np.uint8) * 255  # 白で初期化
for y in range(maze.shape[0]):
    for x in range(maze.shape[1]):
        if maze[y, x] == 1:
            cv2.rectangle(img, (x * cell_size, y * cell_size),
                          ((x + 1) * cell_size, (y + 1) * cell_size),
                          (0, 0, 0), -1)

# 入口と出口を作る
cv2.rectangle(img, (cell_size, cell_size), (2 * cell_size, 2 * cell_size), 255, -1)
cv2.rectangle(img, (width - 2 * cell_size, height - 2 * cell_size),
              (width - cell_size, height - cell_size), 255, -1)

# PGM形式で保存
cv2.imwrite("/maps/maze_map.pgm", img)
print("Maze map saved as maze_map.pgm")
