#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np


class GasMapper(Node):
def __init__(self):
super().__init__('gas_mapper')
self.sub = self.create_subscription(Float32, '/gas_concentration', self.cb, 10)
# Basit 2D grid (x-y düzlemi için placeholder). Gerçek pozisyon verisini Gazebo/TF'den okuyacağız.
self.grid = np.zeros((100, 100), dtype=np.float32)
self.decay = 0.995


def cb(self, msg: Float32):
# Şimdilik sahte bir hücreye yazıyoruz (ileride drone pozisyonuna göre indekslenecek)
i, j = 50, 50
self.grid *= self.decay
self.grid[i, j] = max(self.grid[i, j], msg.data)


if self.get_clock().now().nanoseconds % 50 == 0:
self.get_logger().info(f"c={msg.data:.2f}, cell={self.grid[i,j]:.2f}")




def main():
rclpy.init()
node = GasMapper()
try:
rclpy.spin(node)
except KeyboardInterrupt:
pass
finally:
rclpy.shutdown()


if __name__ == '__main__':
main()