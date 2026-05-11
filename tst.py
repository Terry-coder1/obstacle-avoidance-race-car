import os
import sys

base_path = r"C:\Users\y1394\AppData\Local\Programs\Python\Python313"

os.environ['TCL_LIBRARY'] = os.path.join(base_path, 'tcl', 'tcl8.6')
os.environ['TK_LIBRARY'] = os.path.join(base_path, 'tcl', 'tk8.6')

import tkinter as tk
import math
import random
import time

# --- 配置参数 ---
WIDTH = 800  # 屏幕宽度
HEIGHT = 600  # 屏幕高度
ROBOT_RADIUS = 15  # 机器人半径
SENSOR_COUNT = 180  # 360° 发出多少条传感器射线 (10度一条)
MAX_RANGE = 200  # 超声波最大探测距离
ROBOT_SPEED = 6  # 机器人移动速度
TURN_SPEED = 0.2  # 机器人转向速度 (弧度)


# --- 定义物体 ---
class Obstacle:
    def __init__(self, x1, y1, x2, y2):
        self.x1, self.y1, self.x2, self.y2 = x1, y1, x2, y2


class Robot:
    def __init__(self, x, y, angle):
        self.x = x
        self.y = y
        self.angle = angle  # 机器人当前的朝向 (弧度)


# --- 核心几何计算 ---

def get_intersection(x1, y1, x2, y2, x3, y3, x4, y4):
    """计算两条线段 (x1,y1)-(x2,y2) 和 (x3,y3)-(x4,y4) 的交点"""
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if den == 0: return None  # 平行

    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den

    if 0 <= t <= 1 and 0 <= u <= 1:
        # 有交点
        ix = x1 + t * (x2 - x1)
        iy = y1 + t * (y2 - y1)
        return (ix, iy)
    return None


def get_distance(x1, y1, x2, y2):
    """计算两点间的距离"""
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


# --- 仿真主程序 ---
class RobotSimulation:
    def __init__(self, root):
        self.root = root
        self.root.title("360°超声波机器人避障仿真 (tkinter)")

        # 创建画布
        self.canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="white")
        self.canvas.pack()

        # 初始化物体和机器人
        self.robot = Robot(150, 500, 0)
        self.obstacles = []
        self.create_environment()

        # 用于存储传感器的 UI 对象
        self.sensor_lines = []
        for _ in range(SENSOR_COUNT):
            line = self.canvas.create_line(0, 0, 0, 0, fill="gray", dash=(2, 2))
            self.sensor_lines.append(line)

        # 绘制机器人
        self.robot_ui = self.canvas.create_oval(0, 0, 0, 0, fill="blue", outline="black")
        self.heading_ui = self.canvas.create_line(0, 0, 0, 0, fill="red", width=2)

        # 启动仿真循环
        self.running = True
        self.animate()

    def create_environment(self):
        """创建一个由直道和弯道组成的复合赛道"""
        self.obstacles = []
        track_width = ROBOT_RADIUS * 2 * 2.5  # 宽度依然保持2.5倍

        # 定义赛道的中心路径点 (关键转折点)
        path_points = [
            (100, 100),  # 起点
            (400, 100),  # 第一段直道结束
            (600, 300),  # 弯道至中部
            (400, 500),  # S弯回转
            (100, 500),  # 底部直道
            (100, 100)  # 回到原点形成闭环
        ]

        # 扩展成平滑轨道的简易实现
        def get_walls(p1, p2, width):
            # 计算段向量
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            dist = math.sqrt(dx ** 2 + dy ** 2)
            if dist == 0: return []

            # 单位法向量 (垂直于行进方向)
            nx = -dy / dist
            ny = dx / dist

            # 生成内外四个点
            i1 = (p1[0] - width / 2 * nx, p1[1] - width / 2 * ny)
            i2 = (p2[0] - width / 2 * nx, p2[1] - width / 2 * ny)
            o1 = (p1[0] + width / 2 * nx, p1[1] + width / 2 * ny)
            o2 = (p2[0] + width / 2 * nx, p2[1] + width / 2 * ny)
            return (i1, i2, o1, o2)

        # 遍历路径点生成墙壁
        for i in range(len(path_points) - 1):
            p1 = path_points[i]
            p2 = path_points[i + 1]
            i1, i2, o1, o2 = get_walls(p1, p2, track_width)

            # 添加内墙和外墙
            self.obstacles.append(Obstacle(i1[0], i1[1], i2[0], i2[1]))
            self.obstacles.append(Obstacle(o1[0], o1[1], o2[0], o2[1]))

            # 在拐角处封闭缝隙 (连接处补线)
            if i < len(path_points) - 2:
                next_p2 = path_points[i + 2]
                ni1, ni2, no1, no2 = get_walls(p2, next_p2, track_width)
                self.obstacles.append(Obstacle(i2[0], i2[1], ni1[0], ni1[1]))
                self.obstacles.append(Obstacle(o2[0], o2[1], no1[0], no1[1]))

        # 绘制
        for obs in self.obstacles:
            self.canvas.create_line(obs.x1, obs.y1, obs.x2, obs.y2, fill="black", width=2)


    def update_sensors(self):
        """模拟 360° 超声波数据，计算距离"""
        sensor_data = []  # 存储 (角度, 距离)
        angle_step = (2 * math.pi) / SENSOR_COUNT

        for i in range(SENSOR_COUNT):
            # 计算每条射线的绝对角度
            sensor_angle = self.robot.angle + (i * angle_step)

            # 射线的终点 (假设无障碍)
            end_x = self.robot.x + MAX_RANGE * math.cos(sensor_angle)
            end_y = self.robot.y + MAX_RANGE * math.sin(sensor_angle)

            closest_dist = MAX_RANGE
            closest_point = (end_x, end_y)

            # 检查射线与所有障碍物的交点
            for obs in self.obstacles:
                intersect = get_intersection(self.robot.x, self.robot.y, end_x, end_y,
                                             obs.x1, obs.y1, obs.x2, obs.y2)
                if intersect:
                    dist = get_distance(self.robot.x, self.robot.y, intersect[0], intersect[1])
                    if dist < closest_dist:
                        closest_dist = dist
                        closest_point = intersect

            # 更新传感器 UI (射线颜色随距离变化)
            color = "gray"
            if closest_dist < ROBOT_RADIUS * 2.5:
                color = "red"  # 太近了，变红
            elif closest_dist < MAX_RANGE * 0.6:
                color = "orange"

            self.canvas.coords(self.sensor_lines[i], self.robot.x, self.robot.y, closest_point[0], closest_point[1])
            self.canvas.itemconfig(self.sensor_lines[i], fill=color)

            sensor_data.append((sensor_angle, closest_dist))

        return sensor_data

    def simple_avoid_algorithm(self, sensor_data):
        # 初始化三个方向的力
        f_front_x, f_front_y = 0, 0  # 前方避障力
        f_side_x, f_side_y = 0, 0  # 侧边平衡力

        f_side_x += 0.1 * math.cos(self.robot.angle)
        f_side_y += 0.1 * math.sin(self.robot.angle)

        # 参数设置
        FRONT_THRESHOLD = 70  # 前方探测距离（远，提前反应）
        SIDE_THRESHOLD = 40  # 侧边触发距离（近，靠近边缘才修正）
        SIDE_FORCE_WEIGHT = 0.5  # 侧边力的权重（比前方力小，避免晃动）

        for abs_angle, dist in sensor_data:
            # 计算相对角度
            rel_angle = (abs_angle - self.robot.angle + math.pi) % (2 * math.pi) - math.pi

            # --- 1. 处理前方 45° 范围 (紧急避障) ---
            if abs(rel_angle) <= math.radians(45):
                if dist < FRONT_THRESHOLD:
                    mag = (FRONT_THRESHOLD - dist) / FRONT_THRESHOLD
                    f_front_x += mag * math.cos(abs_angle + math.pi)
                    f_front_y += mag * math.sin(abs_angle + math.pi)

            # --- 2. 处理侧边 45° 到 90° 范围 (边缘平衡) ---
            elif math.radians(45) < abs(rel_angle) <= math.radians(90):
                if dist < SIDE_THRESHOLD:
                    # 侧边斥力：距离越近，推向中间的力越大
                    mag = ((SIDE_THRESHOLD - dist) / SIDE_THRESHOLD) * SIDE_FORCE_WEIGHT
                    # 依然是背离障碍物的方向
                    f_side_x += mag * math.cos(abs_angle + math.pi)
                    f_side_y += mag * math.sin(abs_angle + math.pi)

        # 3. 合并所有的力
        total_x = f_front_x + f_side_x
        total_y = f_front_y + f_side_y

        combined_mag = math.sqrt(total_x ** 2 + total_y ** 2)

        if combined_mag < 0.01:
            return ROBOT_SPEED, 0
        else:
            # 计算最终目标角度
            target_angle = math.atan2(total_y, total_x)
            angle_diff = (target_angle - self.robot.angle + math.pi) % (2 * math.pi) - math.pi
            if (angle_diff < math.pi / 36) & (angle_diff > -math.pi / 36):
                angle_diff = 0
            if (angle_diff > math.pi / 9):
                angle_diff = math.pi / 9
            if (angle_diff < -math.pi / 9):
                angle_diff = -math.pi / 9

            # 转向限制
            turn = max(min(angle_diff, TURN_SPEED), -TURN_SPEED)

            # 动态速度：如果前方力很大，减速；如果只是侧边微调，保持速度
            front_mag = math.sqrt(f_front_x ** 2 + f_front_y ** 2)
            speed_factor = 1.0

            return ROBOT_SPEED * speed_factor, turn

    def animate(self):
        """仿真循环"""
        if not self.running: return

        # 1. 获取传感器数据
        sensor_data = self.update_sensors()

        # 2. 运行避障算法，获取下一步速度和转角
        speed, turn = self.simple_avoid_algorithm(sensor_data)

        # 3. 更新机器人状态
        self.robot.angle += turn
        self.robot.x += speed * math.cos(self.robot.angle)
        self.robot.y += speed * math.sin(self.robot.angle)

        # 4. 更新 UI 显示
        r = ROBOT_RADIUS
        self.canvas.coords(self.robot_ui, self.robot.x - r, self.robot.y - r, self.robot.x + r, self.robot.y + r)

        # 绘制红色的朝向线
        head_x = self.robot.x + (r + 5) * math.cos(self.robot.angle)
        head_y = self.robot.y + (r + 5) * math.sin(self.robot.angle)
        self.canvas.coords(self.heading_ui, self.robot.x, self.robot.y, head_x, head_y)

        # 5. 循环
        self.root.after(20, self.animate)  # 每20毫秒刷新一次 (约50FPS)


# 启动 tkinter
if __name__ == "__main__":
    root = tk.Tk()
    sim = RobotSimulation(root)
    root.main
