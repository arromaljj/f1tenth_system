import math
import csv
from typing import List

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        self.declare_parameter('raceline_csv', '')
        self.declare_parameter('lookahead_distance', 1.0)
        self.declare_parameter('speed', 2.0)

        raceline_file = self.get_parameter('raceline_csv').get_parameter_value().string_value
        self.lookahead = self.get_parameter('lookahead_distance').value
        self.speed = self.get_parameter('speed').value

        self.path = self._load_raceline(raceline_file)
        self.pose = None

        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)

    def _load_raceline(self, filename: str) -> List[tuple]:
        path = []
        if filename:
            try:
                with open(filename, 'r') as f:
                    reader = csv.reader(f)
                    for row in reader:
                        if len(row) >= 2:
                            path.append((float(row[0]), float(row[1])))
            except Exception as e:
                self.get_logger().warning(f'Failed to load raceline: {e}')
        return path

    def odom_callback(self, msg: Odometry):
        self.pose = msg.pose.pose

    def timer_callback(self):
        if self.pose is None or not self.path:
            return
        # find lookahead point
        x = self.pose.position.x
        y = self.pose.position.y
        closest_dist = float('inf')
        closest_index = 0
        for i, (px, py) in enumerate(self.path):
            dist = math.hypot(px - x, py - y)
            if dist < closest_dist:
                closest_dist = dist
                closest_index = i
        # move forward along path until lookahead distance
        path_len = len(self.path)
        d = 0.0
        lookahead_index = closest_index
        while d < self.lookahead and lookahead_index + 1 < path_len:
            x1, y1 = self.path[lookahead_index]
            x2, y2 = self.path[lookahead_index + 1]
            d += math.hypot(x2 - x1, y2 - y1)
            lookahead_index += 1
        goal_x, goal_y = self.path[lookahead_index]

        # transform goal to vehicle frame
        yaw = self._yaw_from_quaternion(self.pose.orientation)
        dx = goal_x - x
        dy = goal_y - y
        local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy
        if local_x == 0:
            curvature = 0.0
        else:
            curvature = 2 * local_y / (local_x ** 2 + local_y ** 2)

        steering_angle = curvature
        msg = AckermannDriveStamped()
        msg.drive.speed = float(self.speed)
        msg.drive.steering_angle = float(steering_angle)
        self.drive_pub.publish(msg)

    def _yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
