class Node:
    def __init__(self, x, y, cost=0, heuristic=0, parent=None):
        self.x = x  # x座標
        self.y = y  # y座標
        self.cost = cost  # 実コスト (g)
        self.heuristic = heuristic  # 推定コスト (h)
        self.parent = parent  # 親ノード

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)
#隣接ノードを取得する関数を追加
    def get_neighbors(self):
        neighbors = []
        # ここでは、上下左右の4方向に移動可能と仮定します。
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            new_x, new_y = self.x + dx, self.y + dy
            # 座標がグリッドの範囲内にあるか確認するバリデーションが必要
            # (ここでは省略)
            neighbors.append(Node(new_x, new_y))
        return neighbors
# 距離を計算する関数を追加
    def distance_to(self, other):
      #ユークリッド距離
      return ((self.x - other.x)**2 + (self.y - other.y)**2)**0.5
def astar(start, goal, heuristic_func):
    open_list = [start]
    closed_list = []

    while open_list:
        # オープンリストからf値が最小のノードを取得
        current_node = min(open_list)
        open_list.remove(current_node)
        closed_list.append(current_node)

        # ゴールに到達したら経路を返す
        if current_node == goal:
            return reconstruct_path(current_node)

        # 隣接ノードを調べる
        neighbors = current_node.get_neighbors()
        for neighbor in neighbors:
            if neighbor in closed_list:
                continue

            # 新しいgスコアを計算
            tentative_g_score = current_node.cost + current_node.distance_to(neighbor)

            if neighbor not in open_list or tentative_g_score < neighbor.cost:
                neighbor.cost = tentative_g_score
                neighbor.heuristic = heuristic_func(neighbor, goal)
                neighbor.parent = current_node

                if neighbor not in open_list:
                    open_list.append(neighbor)

    # 経路が見つからない場合はNoneを返す
    return None
def reconstruct_path(node):
    path = []
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return list(reversed(path))
# スタートとゴールのノードを作成
start_node = Node(0, 0)
goal_node = Node(5, 5)

# ユークリッド距離をヒューリスティック関数として定義
def euclidean_distance(node, goal):
    return ((node.x - goal.x)**2 + (node.y - goal.y)**2)**0.5

# A*アルゴリズムを実行
path = astar(start_node, goal_node, euclidean_distance)

# 結果を表示
if path:
    print("最短経路:", path)
else:
    print("経路が見つかりませんでした。")
