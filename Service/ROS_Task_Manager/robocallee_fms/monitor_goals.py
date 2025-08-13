# monitor_goals.py
import rclpy, math, time
from rclpy.node import Node
from nav_msgs.msg import Odometry

RES = 0.5  # _MAP_RESOLUTION_ 값으로 바꿔줘!
GOALS = { # 로봇ID: (grid_x, grid_y, yaw_rad)
    1: (5, 8, 0.0),
    2: (5, 8, 0.0),
    3: (5, 8, 0.0),
}
POS_TOL = 0.15
YAW_TOL = math.radians(15)

def grid_to_m(x, y):
    mx = ((y - 1) * RES) + (RES / 2.0)
    my = ((x - 1) * RES) + (RES / 2.0)
    return mx, my

class Mon(Node):
    def __init__(self, nrobots):
        super().__init__('goal_monitor')
        self.targets = {rid: (*grid_to_m(*GOALS[rid][:2]), GOALS[rid][2]) for rid in GOALS}
        self.subs = []
        for rid in range(1, nrobots+1):
            self.subs.append(self.create_subscription(
                Odometry, f'/odom_{rid}', lambda msg, rid=rid: self.cb(msg, rid), 10))
    def cb(self, msg, rid):
        tx, ty, tyaw = self.targets[rid]
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        dx, dy = px-tx, py-ty
        dist = math.hypot(dx, dy)
        if dist < POS_TOL:
            self.get_logger().info(f'robot#{rid} REACHED (d={dist:.2f} m)')

def main():
    rclpy.init()
    node = Mon(nrobots=3)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
