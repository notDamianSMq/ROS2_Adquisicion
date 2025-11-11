#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rcl_interfaces.msg import SetParametersResult

from project_interfaces.srv import CleanCloud


class CloudNode(Node):
    def __init__(self):
        super().__init__('cloud_points_node')
        self.subscriber = self.create_subscription(
            PointCloud2, "/ouster/points", self.callback, 10
        )
        self.publisher = self.create_publisher(PointCloud2, "/clean/points", 10)

        self.precision = 3
        self.declare_parameter('precision', 3)
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.client = self.create_client(CleanCloud, 'clean_pointcloud')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servidor...')

        self.get_logger().info('Nodo cloud_points_node iniciado. Escuchando /ouster/points...')

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'precision':
                if param.value < 0 or param.value > 10:
                    #self.get_logger().warn('El parámetro precision debe estar entre 0 y 10')
                    return SetParametersResult(successful=False)
                self.precision = param.value
                #self.get_logger().info(f'Nuevo valor de precision: {self.precision}')
        return SetParametersResult(successful=True)

    def callback(self, msg: PointCloud2):

        self.get_logger().info(
            f'Recibido PointCloud2: ancho={msg.width}, alto={msg.height}, campos={len(msg.fields)}\n'
        )

        self.send_clean_request(msg)

        # Publicar los datos una vez han sido limpiados
        self.get_logger().info("Transmitiendo datos limpiados\n")
        #self.publisher.publish(res)

    def send_clean_request(self, msg: PointCloud2):
        req = CleanCloud.Request()

        req.input = msg
        req.precision = self.get_parameter('precision').value

        future = self.client.call_async(req)
        future.add_done_callback(lambda f: self.publish_cleaned(f))

    def publish_cleaned(self, future):
        try:
            res = future.result()
            self.publisher.publish(res.output)
            self.get_logger().info("Transmitida nube limpia")
        except Exception as e:
            self.get_logger().error(f"Error en servicio: {e}")


def main(args=None):
    try:
        rclpy.init(args=args)
        node = CloudNode()
        rclpy.spin(node)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()