#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from project_interfaces.srv import CleanCloud


class CloudNode(Node):
    def __init__(self):
        super().__init__('cloud_points_node')
        self.subscriber = self.create_subscription(
            PointCloud2, "/ouster/points", self.callback, 10
        )
        self.publisher = self.create_publisher(PointCloud2, "/clean/points", 10)

        self.client = self.create_client(CleanCloud, 'clean_pointcloud')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servidor...')

        self.get_logger().info('Nodo cloud_points_node iniciado. Escuchando /ouster/points...')

    def callback(self, msg: PointCloud2):

        self.get_logger().info(
            f'Recibido PointCloud2: ancho={msg.width}, alto={msg.height}, campos={len(msg.fields)}\n'
        )

        res = self.send_clean_request(msg)

        # Publicar los datos una vez han sido limpiados
        self.get_logger().info("Transmitiendo datos limpiados\n")
        self.publisher.publish(res)

    def send_clean_request(self, msg: PointCloud2):
        req = CleanCloud.Request()

        req.input = msg
        req.precision = 3

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        self.get_logger().info("Recibida nube limpia.")
        return res.output


def main(args=None):
    try:
        rclpy.init(args=args)
        node = CloudNode()
        rclpy.spin(node)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()