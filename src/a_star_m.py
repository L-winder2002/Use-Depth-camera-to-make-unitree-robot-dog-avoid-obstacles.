import math
import numpy as np

class AStarPlanner:
    def __init__(self, ox, oy, resolution, rr, n):
        """
        初始化 A* 网格地图

        ox: 障碍物 x 坐标列表 [m]
        oy: 障碍物 y 坐标列表 [m]
        resolution: 网格分辨率 [m]
        rr: 机器人的半径 [m]
        n: 预留参数（此处不做处理）
        """
        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)
        self.n = n

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # 网格索引
            self.y = y  # 网格索引
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return f"{self.x},{self.y},{self.cost},{self.parent_index}"

    def planning(self, sx, sy, gx, gy):
        """
        A* 路径搜索

        输入:
            sx, sy: 起点坐标 [m]
            gx, gy: 终点坐标 [m]
        输出:
            rx, ry: 路径上各点的 x 和 y 坐标列表
        """
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            # 选择 f(n) = g(n) + h(n) 最小的节点进行扩展
            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node, open_set[o])
            )
            current = open_set[c_id]

            if current.x == goal_node.x and current.y == goal_node.y:
                print("找到目标！")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # 将当前节点从 open_set 中移除，并加入 closed_set
            del open_set[c_id]
            closed_set[c_id] = current

            # 根据运动模型扩展邻近节点
            for i, _ in enumerate(self.motion):
                node = self.Node(
                    current.x + self.motion[i][0],
                    current.y + self.motion[i][1],
                    current.cost + self.motion[i][2],
                    c_id
                )
                n_id = self.calc_grid_index(node)

                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # 发现新节点
                else:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node  # 更新更优路径

        rx, ry = self.calc_final_path(goal_node, closed_set)
        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # 反向回溯，生成最终路径
        rx = [self.calc_grid_position(goal_node.x, self.min_x)]
        ry = [self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index
        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # 启发函数权重
        return w * math.hypot(n1.x - n2.x, n1.y - n2.y)

    def calc_grid_position(self, index, min_position):
        """根据网格索引计算实际坐标"""
        return index * self.resolution + min_position

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return node.y * self.x_width + node.x

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)
        if px < self.min_x or py < self.min_y or px >= self.max_x or py >= self.max_y:
            return False
        # 碰撞检测
        if self.obstacle_map[node.x][node.y]:
            return False
        return True

    def calc_obstacle_map(self, ox, oy):
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)

        # 生成障碍物地图（二维数组）
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for (iox, ioy) in zip(ox, oy):
                    if math.hypot(iox - x, ioy - y) <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # 定义运动模型：dx, dy, 代价
        motion = [
            [1, 0, 1],
            [0, 1, 1],
            [-1, 0, 1],
            [0, -1, 1],
            [-1, -1, math.sqrt(2)],
            [-1, 1, math.sqrt(2)],
            [1, -1, math.sqrt(2)],
            [1, 1, math.sqrt(2)]
        ]
        return motion


class planner():
    # def __init__(self):

    def calculate_obstacle_position(self, left_ratio, right_ratio, depth):
        """
        根据相机视场角计算障碍物在俯视图中的左右边界位置
        left_ratio, right_ratio: 分别为左、右边界比例（范围 0~1）
        depth: 障碍物距离 [m]
        """
        theta = 87.2  # 水平视场角（度）
        half_fov_rad = np.radians(theta / 2)
        l = 2 * depth * np.tan(half_fov_rad)  # 计算深度处的水平边长
        left_position = l * (left_ratio - 0.5)
        right_position = l * (right_ratio - 0.5)
        return left_position, right_position, l

    def rotate_point(self,x, y, origin_x, origin_y, theta):
        """ 旋转坐标点 (x, y) 绕 (origin_x, origin_y) 逆时针旋转 theta 角度 """
        x_shifted = x - origin_x
        y_shifted = y - origin_y
        x_rotated = x_shifted * math.cos(theta) - y_shifted * math.sin(theta)
        y_rotated = x_shifted * math.sin(theta) + y_shifted * math.cos(theta)

        return x_rotated + origin_x, y_rotated + origin_y

    def path_planner(self, origin_x, origin_y, coordinate_box,yaw,gx,gy,obstacle_x,obstacle_y):
        grid_size = 0.5    # [m] 网格大小
        robot_radius = 0.3   # [m] 机器狗半径
        ox,oy=[],[]
        top,left,right,bottom=0,0,0,0
        if origin_x>gx:   #根据起点终点设定边界框，保障起点终点最近离边框10m
            left=gx-10
            right=origin_x+10
        else:
            left=origin_x-10
            right=gx+10
        if origin_y>gy:
            top = origin_y + 10
            bottom = gy - 10
        else:
            top = gy + 10
            bottom = origin_y - 10

        for i in range(int(left), int(right)):    #绘制top框
            ox.append(i)
            oy.append(top)
        for i in range(int(left), int(right)):   #绘制bottom框
            ox.append(i)
            oy.append(bottom)
        for i in range(int(bottom), int(top)):    #绘制right框
            ox.append(right)
            oy.append(i)
        for i in range(int(bottom), int(top)):   #绘制left框
            ox.append(left)
            oy.append(i)

        # 绘制通过 bounding box 计算得到的障碍物
        for n in range(len(coordinate_box)):
            # 此处传入的参数为比例和深度，例如 [left_ratio, right_ratio, depth]
            left_ratio, right_ratio, depth = coordinate_box[n]
            left_position, right_position, l_val = self.calculate_obstacle_position(left_ratio, right_ratio, depth)
            # 为保证障碍物能被绘制，使用 math.floor 和 math.ceil 计算边界
            x_start = math.floor(left_position + origin_x)
            x_end = math.ceil(right_position + origin_x)
            y_start = math.floor(depth + origin_y)
            y_end = math.ceil(depth + origin_y + 0.2)  # 设定障碍物厚度为 0.2 m
            step = 0.7  # 设定填充步长
            for i in np.arange(x_start, x_end,step):
                for j in np.arange(y_start, y_end,step):
                    # 旋转当前网格坐标 (i, j) 到新的坐标
                    x_rot, y_rot = self.rotate_point(i, j, origin_x, origin_y, yaw-1.57)  #-1.57的原因是障碍物排列默认朝y轴正方向，将其顺时针1.57弧度（90度）之后朝向机器狗实际的x正方向
                    obstacle_x.append(round(x_rot / step) * step)
                    obstacle_y.append(round(y_rot / step) * step)
        if len(obstacle_x)>50:
            obstacle_x=obstacle_x[len(obstacle_x)-51:len(obstacle_x)-1]
            obstacle_y=obstacle_y[len(obstacle_y)-51:len(obstacle_y)-1]
        ox=ox+obstacle_x
        oy=oy+obstacle_y
        # 调用 A* 规划路径
        a_star = AStarPlanner(ox, oy, grid_size, robot_radius, len(coordinate_box))
        rx, ry = a_star.planning(origin_x, origin_y, gx, gy)
        return rx, ry ,ox ,oy

