import numpy

import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.animation import FuncAnimation

class Maze:
    def __init__(self, map_size=(10,10), object_num=25):
        self.map_size = map_size
        self.object_num = object_num
        self.maze = [[0] * self.map_size[0] for i in range(self.map_size[1])]
        pass

    def generation_map(self, object_list, start, goal):

        # set cost of maze outline
        for i in range(self.map_size[1]):
            self.maze[0][i] = object_cost
            self.maze[self.map_size[0] - 1][i] = object_cost

        for j in range(self.map_size[0]):
            self.maze[j][0] = object_cost
            self.maze[j][self.map_size[1] - 1] = object_cost

        # set cost of objects (like wall...). Number of object is set at first
        if object_list:
            for object in object_list:
                self.maze[object[0]][object[1]] = object_cost

        # set start&goal position with position class
        self.goal = goal
        self.start = start
        # set cost
        self.maze[start.x][start.y] = -1
        self.maze[goal.x][goal.y] = 1
        # set full cost
        self.goal.set_ini_cost(self.goal)
        self.start.set_ini_cost(self.goal)
        return self.maze, self.start, self.goal

    def twice_graph(self, route_list, trace_list):
        fig = plt.figure(figsize=self.map_size)

        #グラフを描画するsubplot領域を作成。
        ax1 = fig.add_subplot(1, 2, 1)
        ax2 = fig.add_subplot(1, 2, 2)


        ax1.set_xticks(list(range(self.map_size[0]+1)))
        ax1.set_yticks(list(range(self.map_size[1]+1)))
        ax1.grid()
        ax1.set_aspect('equal')


        ax2.set_xticks(list(range(self.map_size[0]+1)))
        ax2.set_yticks(list(range(self.map_size[1]+1)))
        ax2.grid()
        ax2.set_aspect('equal')


        # for before
        for x in range(self.map_size[0]):
                for y in range(self.map_size[1]):
                    if self.maze[x][y] == object_cost:
                        r = patches.Rectangle( xy=(x,y), width=1, height=1, color="grey", alpha=0.5) # 四角形のオブジェクト
                        # 4. Axesに図形オブジェクト追加・表示
                        ax1.add_patch(r)

        # # show start&goal position with box
        # r = patches.Rectangle( xy=(self.start.x,self.start.y), width=1, height=1, color="blue", alpha=0.5) # 四角形のオブジェクト
        # ax1.add_patch(r)
        # r = patches.Rectangle( xy=(self.goal.x,self.goal.y), width=1, height=1, color="red", alpha=0.5) # 四角形のオブジェクト
        # ax1.add_patch(r)
        for p in trace_list:
            ax1.text(p.x + 0.1, p.y + 0.1, round(p.h,1))

        # show start&goal position with star
        s2g = [[self.start.convert_coordinate().x, self.goal.convert_coordinate().x],
                 [self.start.convert_coordinate().y, self.goal.convert_coordinate().y]]

        ax1.scatter(s2g[0], s2g[1], c="orange", marker="*", s=350)
        ax2.scatter(s2g[0], s2g[1], c="orange", marker="*", s=350, zorder=5)

        # for after
        for x in range(self.map_size[0]):
            for y in range(self.map_size[1]):
                if self.maze[x][y] == object_cost:
                    r = patches.Rectangle( xy=(x,y), width=1, height=1, color="grey",alpha=0.5) # 四角形のオブジェクト
                    # 4. Axesに図形オブジェクト追加・表示
                    ax2.add_patch(r)

        # show all node explored with blue x
        for i in range(len(trace_list)):
            ax2.scatter(trace_list[i].convert_coordinate().x, trace_list[i].convert_coordinate().y, c="blue", marker="x")

        # # show all node explored with yellow box
        # for i in range(len(trace_list)):
        #     r = patches.Rectangle( xy=(trace_list[i].x, trace_list[i].y), width=1, height=1, color="yellow", alpha=0.5) # 四角形のオブジェクト
        #     # 4. Axesに図形オブジェクト追加・表示
        #     ax2.add_patch(r)

        # show route node with red x
        for i in range(len(route_list)):
            ax2.scatter(route_list[i].convert_coordinate().x, route_list[i].convert_coordinate().y, c="red", marker="x")

        # # show route node with red green box
        # for i in range(len(route_list)):
        #     r = patches.Rectangle( xy=(route_list[i].x, route_list[i].y), width=1, height=1, color="green", alpha=0.5) # 四角形のオブジェクト
        #     # 4. Axesに図形オブジェクト追加・表示
        #     ax2.add_patch(r)

        # # show start&goal position with box
        # r = patches.Rectangle( xy=(self.start.x,self.start.y), width=1, height=1, color="blue", alpha=0.5) # 四角形のオブジェクト
        # ax2.add_patch(r)
        # r = patches.Rectangle( xy=(self.goal.x,self.goal.y), width=1, height=1, color="red", alpha=0.5) # 四角形のオブジェクト
        # ax2.add_patch(r)

        plt.show()

    def set_maze_graph(self):
        fig, ax = plt.subplots(figsize=self.map_size)

        ax.set_xticks(list(range(self.map_size[0]+1)))
        ax.set_yticks(list(range(self.map_size[1]+1)))
        ax.grid()
        ax.set_aspect('equal')

        # set maze
        for x in range(self.map_size[0]):
                for y in range(self.map_size[1]):
                    if self.maze[x][y] == object_cost:
                        r = patches.Rectangle( xy=(x,y), width=1, height=1, color="grey", alpha=0.5) # 四角形のオブジェクト
                        # 4. Axesに図形オブジェクト追加・表示
                        ax.add_patch(r)
        # set star&goal position
        s2g = [[self.start.convert_coordinate().x, self.goal.convert_coordinate().x],
                 [self.start.convert_coordinate().y, self.goal.convert_coordinate().y]]

        ax.scatter(s2g[0], s2g[1], c="orange", marker="*", s=350)
        return fig, ax

    def animation(self, frame):
        ax.clear()
        # draw maze
        for x in range(self.map_size[0]):
                for y in range(self.map_size[1]):
                    if self.maze[x][y] == object_cost:
                        r = patches.Rectangle( xy=(x,y), width=1, height=1, color="grey", alpha=0.5) # 四角形のオブジェクト
                        # 4. Axesに図形オブジェクト追加・表示
                        ax.add_patch(r)
        # set start&goal
        s2g = [[self.start.convert_coordinate().x, self.goal.convert_coordinate().x],
                 [self.start.convert_coordinate().y, self.goal.convert_coordinate().y]]

        ax.scatter(s2g[0], s2g[1], c="orange", marker="*", s=350)
        plt.text(self.start.x + 0.7, self.start.y + 0.7, "S")
        plt.text(self.goal.x + 0.7, self.goal.y + 0.7, "G")

        # trace the route
        for i in range(frame+1):
            ax.scatter(trace_list[i].convert_coordinate().x, trace_list[i].convert_coordinate().y, c="blue", marker="x", zorder=0)

        # show route list at the end
        if frame+1 == len(trace_list):
            for i in range(1,len(route_list)-1):
                ax.scatter(route_list[i].convert_coordinate().x, route_list[i].convert_coordinate().y, c="red", marker="x", zorder=1)

        ax.set_xlim([0, self.map_size[0]])
        ax.set_ylim([0, self.map_size[1]])

        ax.set_xticks(list(range(self.map_size[0]+1)))
        ax.set_yticks(list(range(self.map_size[1]+1)))
        ax.grid()
        ax.set_aspect('equal')

class Position:
    def __init__(self, x, y, depth=0, parent_x=-1, parent_y=-1):
        self.x = x                  # x座標
        self.y = y                  # y座標
        self.depth = depth          # 移動回数
        self.parent_x = parent_x    # 一つ前に通ったマスのx座標
        self.parent_y = parent_y    # 一つ前に通ったマスのy座標

        # cost for A* algorithm
        self.g = self.depth         # cost from the start to the current
        # cost from the current to the goal
        self.h = 0
        self.f = self.g + self.h    # total cost

    def set_ini_cost(self, goal):
        # cost for A* algorithm
        self.g = self.depth         # cost from the start to the current
        # cost from the current to the goal
        self.h = numpy.sqrt((goal.x -self.x)**2 + (goal.y - self.y)**2)
        self.f = self.g + self.h    # total cost

    def cal_cost(self):
        # cost for A* algorithm
        self.g = self.depth         # cost from the start to the current
        # cost from the current to the goal
        self.h = numpy.sqrt((goal.x -self.x)**2 + (goal.y - self.y)**2)
        self.f = self.g + self.h    # total cost

    def add_around(self):
        x, y, depth = self.x, self.y, self.depth
        left_pos = Position(x-1, y, depth+1, x, y)
        left_pos.cal_cost()
        # print(left_pos.h)
        right_pos = Position(x+1, y, depth+1, x, y)
        right_pos.cal_cost()
        # print(right_pos.h)
        up_pos = Position(x, y-1, depth+1, x, y)
        up_pos.cal_cost()
        down_pos = Position(x, y+1, depth+1, x, y)
        down_pos.cal_cost()

        around_list = []

        around_list.extend([left_pos, right_pos, up_pos, down_pos])
        return around_list

    def convert_coordinate(self):
        graph_coordinate = Position(self.x+0.5, self.y+0.5)
        return graph_coordinate

def RouteRecord(trace_list):
    # get number of all traced position to achieve the goal
    num_trace = len(trace_list)
    # num_goal = num_trace - 1
    n = num_trace - 1
    route_list = []
    cnt = 0
    while True:
        child = trace_list[n]
        route_list.append(child)
        # print(cnt)
        if (cnt>len(trace_list)):
            print("Error: count is over list length")
            return route_list
        else:
            cnt = cnt+1
            if child.parent_x > 0:
                # find the list number of goal's parent position and update "n" to it
                for i in range(num_trace):
                    p = trace_list[i]
                    if p.x == child.parent_x and p.y == child.parent_y:
                            n = i
                            break
                    else:
                        continue

            else:
                break
    return route_list


# 探索関数：ゴールしたらそのときの位置・移動数を返す
def Search(start):
    # スタート位置（x座標, y座標, 移動回数）をセット
    ## this can be called as open list
    explore_list = [start]
    # set list to trace path
    ## this can be called as closed list
    trace_list = [start]
    route_list = []

    # while len(explore_list) > 0:# 探索可能ならTrue(listが空ならfalse)
    while len(explore_list)>0:
        # リストから探索する位置を取得
        ## the point with smallest cost should be explored
        f_min = 100000
        for index, listed in enumerate(explore_list):
            if listed.f < f_min:
                # print("HI")
                f_min = listed.f
                last_point = explore_list.pop(index)
            else:
                continue

        # update explore list
        around_list = last_point.add_around()
        for i in range(len(around_list)):
            p = around_list[i]
            # print(p.h)
            # if the point is outside of maze or on object, skip
            if maze[p.x][p.y] == object_cost:
                continue
            # if the point is start, skip
            elif maze[p.x][p.y] == -1:
                continue
            # if the point is already on the trace list, compare the cost
            ## if the cost of the point in current route is smaller than that of listed one,
            ## remove listed one from trace list, and add the current to explore&trace list as new
            elif maze[p.x][p.y] == 2:
                # find the point in trace_list
                for listed in trace_list:
                    if listed.x == p.x and listed.y == p.y:
                        # compare the cost
                        if p.f < listed.f: # if current route cost smaller
                            trace_list.remove(listed) # remove listed one
                            # add the current to explore&trace list
                            explore_list.append(p)
                            trace_list.append(p)
                        else:
                            continue

            # if the point is goal, FINISH
            elif maze[p.x][p.y] == 1:
                print("Find goal & Record it")
                trace_list.append(p)
                print(len(trace_list))
                route_list = RouteRecord(trace_list)
                # route_list = []
                return route_list, trace_list

            # if the point should be explored, add to the explore list
            elif maze[p.x][p.y] == 0:
                maze[p.x][p.y] = 2
                # update explore list
                explore_list.append(p)
                trace_list.append(p)

            # if the point is arounded by objects&explored point, skip
            else:
                print("No route to go")
                continue

    print("No route was found")
    route_list=[], trace_list=[]
    return route_list, trace_list

if __name__ == '__main__':
    object_cost = 100
    object_list = [(2,2), (2,3), (3,2), (3,3), (6,2), (6,3), (7,2), (7,3), (6,6), (6,7), (7,6), (7,7)]
    # object_list = [(4,4), (5,4), (6,4), (4,5), (5,5), (6,5), (4,6), (5,6), (6,6)]
    start = Position(1, 1, 0)
    goal = Position(8, 8, 0)

    create_maze = Maze()
    maze, start, goal = create_maze.generation_map(object_list, start, goal) # generate map
    fig, ax = create_maze.set_maze_graph()
    route_list, trace_list= Search(start)  # 探索
    # Check if the return value Search func returned
    ## if 0 was returned, not route was found
    if route_list == [] and trace_list == []:
        print("Try again")
    else:
        create_maze.twice_graph(route_list, trace_list)
        # アニメーション化
        anim = FuncAnimation(fig, create_maze.animation, frames=len(trace_list), interval=100, repeat=False)
        anim.save("test.gif", writer="pillow")
        plt.show()
