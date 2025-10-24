#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

# Import our custom message
from project_interfaces.msg import ObstacleData
# We will create a custom service later, but for now, let's use a standard one
from std_srvs.srv import SetBool

class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')

        # 1. PARAMETER: Define a configurable detection threshold
        param_descriptor = ParameterDescriptor(description='Distance (m) to trigger obstacle detection.')
        self.declare_parameter('detection_threshold', 2.0, param_descriptor)
        self.is_obstacle_detected_ = False

        # 2. SUBSCRIBER: Listen to the /obstacle_data topic
        self.subscriber_ = self.create_subscription(
            ObstacleData,
            'obstacle_data',
            self.obstacle_data_callback,
            10)

        # 3. SERVICE: Provide a service to report status
        # We'll use a simple service for now. A request 'True' can mean "is there an obstacle?"
        # A response 'True' will mean "Yes, there is."
        self.service_ = self.create_service(
            SetBool,
            'get_obstacle_status',
            self.get_obstacle_status_callback)
        
        self.get_logger().info('Obstacle Detector node started.')

    def obstacle_data_callback(self, msg):
        # Get the current threshold value from the parameter
        threshold = self.get_parameter('detection_threshold').get_parameter_value().double_value

        # The actual detection logic
        if msg.is_obstacle_present and msg.distance < threshold:
            self.is_obstacle_detected_ = True
            self.get_logger().warn(f'OBSTACLE DETECTED at {msg.distance:.2f}m (Threshold: {threshold}m)')
        else:
            self.is_obstacle_detected_ = False
            self.get_logger().info(f'Clear. (Distance: {msg.distance:.2f}m, Threshold: {threshold}m)')

    def get_obstacle_status_callback(self, request, response):
        # The request 'data' field is a boolean. We can ignore it for this service.
        # The response 'success' field is a boolean. We use it to report the status.
        response.success = self.is_obstacle_detected_
        
        if self.is_obstacle_detected_:
            response.message = "Obstacle detected!"
        else:
            response.message = "All clear."
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()