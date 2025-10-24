#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from project_interfaces.msg import ObstacleData

class SensorProcessorNode(Node):
    def __init__(self):
        super().__init__('sensor_processor_node')
        
        # Create a publisher on the /obstacle_data topic
        self.publisher_ = self.create_publisher(ObstacleData, 'obstacle_data', 10)
        
        # Create a timer that calls the publish_data callback every 1 second
        self.timer_ = self.create_timer(1.0, self.publish_data_callback)
        self.get_logger().info('Sensor Processor node started, publishing dummy data...')

    def publish_data_callback(self):
        # Create a new message object
        msg = ObstacleData()
        
        # Fill the message with dummy data
        msg.is_obstacle_present = True
        msg.distance = 10.0  # Default safe distance
        
        # Publish the message
        self.publisher_.publish(msg)
        
        # Log to the console
        self.get_logger().info(f'Publishing: [Obstacle: {msg.is_obstacle_present}, Distance: {msg.distance}]')

def main(args=None):
    rclpy.init(args=args)
    node = SensorProcessorNode()
    rclpy.spin(node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()