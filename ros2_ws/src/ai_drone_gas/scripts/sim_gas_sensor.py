#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import math

class SimGasSensor(Node):
    def __init__(self):
        super().__init__('sim_gas_sensor')
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.pub = self.create_publisher(Float32, '/gas_concentration', 10)
        # (x, y, peak, radius) — world’deki 3 kaynakla uyumlu
        self.sources = [
            (-4.0, -3.65, 80.0, 3.0),  # L1
            ( 2.0,  3.65, 40.0, 3.0),  # R1
            ( 9.0, -3.65, 20.0, 3.0),  # L2
        ]

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        total = 0.0
        for sx, sy, peak, R in self.sources:
            d2 = (p.x - sx)**2 + (p.y - sy)**2
            total += peak * math.exp(-d2 / (2 * R * R))
        self.pub.publish(Float32(data=float(total)))

def main():
    rclpy.init()
    node = SimGasSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
