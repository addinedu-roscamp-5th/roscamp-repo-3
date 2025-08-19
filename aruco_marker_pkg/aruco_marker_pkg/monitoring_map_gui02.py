#!/usr/bin/env python3
"""
Multi-Robot Monitor GUI for 20cm Resolution Traffic Planner
- Map: 12x7 cells (10x5 + border), 20cm resolution
- Coordinate range: (-0.2, -0.2) to (2.2, 1.2)
- Real map area: (0, 0) to (2.0, 1.0)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math
import threading
import sys
import numpy as np

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout
from PyQt5.QtCore import QTimer
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.patches import Patch
from matplotlib.lines import Line2D

NUM_ROBOTS = 3  # 최대 로봇 수

# ✅ 20cm 해상도에 맞게 조정된 맵 (12x7 크기, 테두리 포함)
# TrafficPlannerNode02.cpp와 정확히 동일한 맵
custom_map = [
    [1,1,1,1,1,1,1,1,1,1,1,1],  # 테두리 (row 0)
    [1,0,0,0,1,0,0,0,0,0,0,1],  # 자유공간 (row 1) ✅
    [1,0,0,0,1,0,0,0,0,0,0,1],  # 장애물 있음 (row 2)
    [1,0,0,0,1,0,0,1,1,0,0,1],  # 장애물 있음 (row 3)
    [1,0,0,0,1,0,0,0,0,0,0,1],  # 장애물 있음 (row 4)
    [1,0,0,0,0,0,0,0,0,0,0,1],  # 자유공간 (row 5) ✅
    [1,1,1,1,1,1,1,1,1,1,1,1]   # 테두리 (row 6)
]

class RobotMonitor(Node):
    def __init__(self):
        super().__init__('multi_robot_monitor_20cm')
        self.robot_poses = {}  # {id: (x, y, theta)}
        self.robot_goals = {}  # {id: (x, y)}
        self.robot_paths = {}  # {id: [(x1, y1), (x2, y2), ...]}
        self.robot_status = {}  # {id: 'active'/'completed'/'timeout'}

        for i in range(1, NUM_ROBOTS + 1):
            # 로봇 상태 초기화
            self.robot_status[i] = 'inactive'
            
            # Odometry 구독 (ArUco 마커 위치)
            self.create_subscription(
                Odometry, f'/odom_{i}', 
                self.make_odom_cb(i), 10)
            
            # Goal 구독 (현재 목표점)
            self.create_subscription(
                PoseStamped, f'/goalpose{i}', 
                self.make_goal_cb(i), 10)
            
            # Path 구독 (전체 경로)
            self.create_subscription(
                Path, f'/path{i}', 
                self.make_path_cb(i), 10)

    def make_odom_cb(self, robot_id):
        def callback(msg):
            pose = msg.pose.pose
            self.robot_poses[robot_id] = (
                pose.position.x, pose.position.y, self.yaw_from_quat(pose.orientation)
            )
            if self.robot_status[robot_id] == 'inactive':
                self.robot_status[robot_id] = 'active'
        return callback

    def make_goal_cb(self, robot_id):
        def callback(msg):
            pose = msg.pose
            self.robot_goals[robot_id] = (pose.position.x, pose.position.y)
            if self.robot_status[robot_id] == 'inactive':
                self.robot_status[robot_id] = 'active'
        return callback

    def make_path_cb(self, robot_id):
        def callback(msg):
            path_points = []
            for pose in msg.poses:
                x = pose.pose.position.x
                y = pose.pose.position.y
                path_points.append((x, y))
            self.robot_paths[robot_id] = path_points
            if self.robot_status[robot_id] == 'inactive':
                self.robot_status[robot_id] = 'active'
            self.get_logger().info(f'🛤️  로봇 {robot_id} 경로 수신: {len(path_points)} 점 (20cm 해상도)')
        return callback

    def yaw_from_quat(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def ros_spin(node):
    rclpy.spin(node)

class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.setWindowTitle("Multi-Robot Monitor (20cm Resolution) - TrafficPlanner02")
        self.node = node

        # ✅ 창 크기 조정 (2:1 비율에 맞게)
        self.resize(1400, 700)

        # ✅ 20cm 해상도 맵을 numpy로 변환 (12x7)
        self.map_array = np.array(custom_map, dtype=np.uint8)
        self.map_array = (1 - self.map_array) * 255  # 0 → 255 (white), 1 → 0 (black)
        self.map_array = np.flipud(self.map_array)   # y축 상하반전

        # ✅ 20cm 해상도 설정
        self.map_resolution = 0.2  # 20cm/cell
        self.map_width = 2.4      # 2.4m (12 cells × 0.2m)
        self.map_height = 1.4     # 1.4m (7 cells × 0.2m)

        # UI 설정
        main_widget = QWidget()
        layout = QVBoxLayout()
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)
        main_widget.setLayout(layout)
        self.setCentralWidget(main_widget)

        self.ax = self.figure.add_subplot(111)

        # 업데이트 타이머
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)  # 100ms 간격

    def update_plot(self):
        self.ax.clear()

        # ✅ 20cm 해상도 맵 출력: 범위 (-0.2, -0.2) ~ (2.2, 1.2)
        extent = [-0.2, 2.2, -0.2, 1.2]
        self.ax.imshow(self.map_array, cmap='gray', origin='lower', extent=extent)

        # ✅ 그리드 라인 표시 (20cm 간격) - 확장된 범위
        for i in range(13):  # -0.2, 0, 0.2, 0.4, ..., 2.2
            x = -0.2 + i * 0.2
            self.ax.axvline(x, color='lightblue', alpha=0.3, linewidth=0.5)
        for j in range(8):   # -0.2, 0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2
            y = -0.2 + j * 0.2
            self.ax.axhline(y, color='lightblue', alpha=0.3, linewidth=0.5)

        # ✅ 각 로봇의 경로를 30% 투명도로 표시
        for rid in range(1, NUM_ROBOTS + 1):
            if rid in self.node.robot_paths and len(self.node.robot_paths[rid]) > 1:
                path_points = self.node.robot_paths[rid]
                x_coords = [point[0] for point in path_points]
                y_coords = [point[1] for point in path_points]
                
                # 경로를 선으로 연결하여 표시 (30% 투명도)
                self.ax.plot(x_coords, y_coords, '-', 
                           color=f'C{rid}', linewidth=2, alpha=0.3, 
                           label=f'Robot {rid} Path')
                
                # 웨이포인트 표시 (작은 점들)
                self.ax.plot(x_coords, y_coords, 'o', 
                           color=f'C{rid}', markersize=4, alpha=0.6)
                
                # 경로 시작점과 끝점 표시
                if len(path_points) > 0:
                    # 시작점 (큰 원)
                    self.ax.plot(x_coords[0], y_coords[0], 'o', 
                               color=f'C{rid}', markersize=8, alpha=0.8,
                               markeredgecolor='black', markeredgewidth=1)
                    # 끝점 (사각형)
                    self.ax.plot(x_coords[-1], y_coords[-1], 's', 
                               color=f'C{rid}', markersize=8, alpha=0.8,
                               markeredgecolor='black', markeredgewidth=1)

        # ✅ 로봇 현재 위치와 목표 위치 표시
        for rid in range(1, NUM_ROBOTS + 1):
            if rid in self.node.robot_poses:
                x, y, theta = self.node.robot_poses[rid]
                dx = 0.08 * math.cos(theta)  # 20cm 해상도에 맞게 화살표 크기 조정
                dy = 0.08 * math.sin(theta)
                
                # 로봇 현재 위치 (화살표)
                self.ax.arrow(x, y, dx, dy, head_width=0.02, 
                            color=f'C{rid}', alpha=1.0, linewidth=2)
                # 로봇 현재 위치 (점)
                self.ax.plot(x, y, 'o', color=f'C{rid}', markersize=10, 
                           markeredgecolor='black', markeredgewidth=1)

            if rid in self.node.robot_goals:
                gx, gy = self.node.robot_goals[rid]
                # 현재 목표점 (X 표시)
                self.ax.plot(gx, gy, 'X', color=f'C{rid}', markersize=8, 
                           markeredgecolor='black', markeredgewidth=1)

        # ✅ Legend 추가
        legend_elements = []
        
        # 로봇별 경로 범례
        for rid in range(1, NUM_ROBOTS + 1):
            if rid in self.node.robot_paths and len(self.node.robot_paths[rid]) > 0:
                legend_elements.append(Line2D([0], [0], color=f'C{rid}', linewidth=2, alpha=0.3, 
                                            label=f'Robot {rid} Path'))
        
        # 공통 요소들
        legend_elements.extend([
            Line2D([0], [0], marker='o', color='gray', linewidth=0, markersize=8,
                   markeredgecolor='black', markeredgewidth=1, label='Start Point'),
            Line2D([0], [0], marker='s', color='gray', linewidth=0, markersize=8,
                   markeredgecolor='black', markeredgewidth=1, label='Goal Point'),
            Line2D([0], [0], marker='o', color='gray', linewidth=0, markersize=4,
                   alpha=0.6, label='Waypoints'),
            Line2D([0], [0], marker='X', color='gray', linewidth=0, markersize=8,
                   markeredgecolor='black', markeredgewidth=1, label='Current Goal'),
            Line2D([0], [0], marker='o', color='gray', linewidth=0, markersize=10,
                   markeredgecolor='black', markeredgewidth=1, label='Current Position')
        ])
        
        # Legend 표시 (우상단에 배치)
        self.ax.legend(handles=legend_elements, loc='upper right', bbox_to_anchor=(1.2, 1.0), 
                      framealpha=0.9, fontsize=9)

        # ✅ 좌표축 설정 (-0.2 ~ 2.2, -0.2 ~ 1.2)
        self.ax.set_xlim(-0.2, 2.2)
        self.ax.set_ylim(-0.2, 1.2)
        
        # 20cm 간격으로 주요 눈금 표시
        self.ax.set_xticks(np.arange(-0.2, 2.3, 0.2))
        self.ax.set_yticks(np.arange(-0.2, 1.3, 0.2))
        
        # 10cm 간격으로 보조 눈금 표시
        self.ax.set_xticks(np.arange(-0.2, 2.25, 0.1), minor=True)
        self.ax.set_yticks(np.arange(-0.2, 1.25, 0.1), minor=True)

        self.ax.set_title("Multi-Robot Monitor (20cm Resolution) - 2.4m x 1.4m Map")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.grid(True, alpha=0.3)
        
        self.canvas.draw()

def main(args=None):
    rclpy.init(args=args)
    node = RobotMonitor()

    # ROS2 스핀을 별도 스레드에서 실행
    spin_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    spin_thread.start()

    # PyQt5 애플리케이션 실행
    app = QApplication(sys.argv)
    window = MainWindow(node)
    window.show()
    
    try:
        ret = app.exec_()
    except KeyboardInterrupt:
        print("\n종료 중...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(ret)

if __name__ == '__main__':
    main()