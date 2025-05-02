#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
import random
import time

class StateManagerNode(Node):
    """
    State Manager node that manages robot states and system coordination.
    Currently a test implementation that publishes random topics.
    """
    
    def __init__(self):
        super().__init__('state_manager')
        
        # create_publisher(
            # Message Type, 
            # Topic Name(topic that other nodes will subscribe to), 
            # Queue Size (aka. QoS Profile Depth or how many messages can be in the queue)
        # )
        self.state_publisher = self.create_publisher(
            String, 'smore_bot/state', 10)
            
        self.health_publisher = self.create_publisher(
            Bool, 'smore_bot/system_health', 10)
            
        self.battery_publisher = self.create_publisher(
            Float32, 'smore_bot/battery_level', 10)
            
        self.temperature_publisher = self.create_publisher(
            Float32, 'smore_bot/temperature', 10)
        
        # Set up timer for publishing, allows for publishing at a fixed rate
        # create_timer(
          # Time interval in seconds,
          # Callback
        # )
        self.timer = self.create_timer(1.0, self.publish_data)
        
        # Available states for testing
        self.states = ["IDLE", "MOVING", "PROCESSING", "ERROR", "STANDBY"]
        
        # Returns a Logger instance that's associated with this node
        # I believe the options are INFO, DEBUG, WARN, ERROR, FATAL
        # you can configure logging level which is why this matters
        # self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        
        self.get_logger().info('State Manager node has started')
        
    def publish_data(self):
        """Publish test data on various topics"""
        
        # Publish random state
        state_msg = String()
        state_msg.data = random.choice(self.states)
        self.state_publisher.publish(state_msg)
        
        # Publish system health
        health_msg = Bool()
        health_msg.data = random.random() > 0.1  # 90% chance of being healthy
        self.health_publisher.publish(health_msg)
        
        # Publish fake battery level (0-100)
        battery_msg = Float32()
        battery_msg.data = random.uniform(60.0, 100.0)
        self.battery_publisher.publish(battery_msg)
        
        # Publish fake temperature
        temp_msg = Float32()
        temp_msg.data = random.uniform(30.0, 104.0)
        self.temperature_publisher.publish(temp_msg)
        
        self.get_logger().info(f'Published state: {state_msg.data}, health: {health_msg.data}, '
                              f'battery: {battery_msg.data:.1f}%, temp: {temp_msg.data:.1f}°F')


def main(args=None):
    rclpy.init(args=args) # Initialize the Python client library
    node = StateManagerNode() # Create an instance of our node
    
    try:
        rclpy.spin(node) # This starts the node and keeps it running until the node is shut down
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        node.destroy_node()  # Clean shit up
        rclpy.shutdown()


if __name__ == '__main__':
    main()
