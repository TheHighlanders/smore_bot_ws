#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class SmoreBotController(Node):
    def __init__(self):
        super().__init__("smore_bot_controller")

        # Create an action client for the joint trajectory controller
        self.action_client = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")

        # Wait for the action server to be available
        self.get_logger().info("Waiting for action server...")
        self.action_client.wait_for_server()
        self.get_logger().info("Action server connected!")

        # Joint names in the correct order
        self.joint_names = ["base_to_shoulder", "shoulder_to_upper_arm", "elbow", "wrist"]

    def send_joint_trajectory(self, positions, duration_sec=1.0):
        """Send a joint trajectory to the robot"""
        self.get_logger().info(f"Sending trajectory to positions: {positions}")

        # Create a joint trajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        # Create a trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(self.joint_names)
        point.accelerations = [0.0] * len(self.joint_names)

        # Set the time from start
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec - int(duration_sec)) * 1e9))

        # Add the point to the trajectory
        trajectory_msg.points.append(point)

        # Create the action goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory_msg

        # Send the goal and wait for the result
        self.get_logger().info("Sending goal...")
        send_goal_future = self.action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return False

        self.get_logger().info("Goal accepted!")

        # Wait for the result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        result = get_result_future.result().result
        if result.error_code != result.SUCCESSFUL:
            self.get_logger().error(f"Goal failed with error code: {result.error_code}")
            return False

        self.get_logger().info("Goal succeeded!")
        return True

    def run_demo(self):
        """Run a simple demo of robot movements"""
        self.get_logger().info("Starting demo sequence...")

        # Home position
        self.send_joint_trajectory([0.0, 0.0, 0.0, 0.0], 2.0)
        time.sleep(2.0)

        # Position 1
        self.send_joint_trajectory([1.0, 0.5, -0.5, 0.8], 3.0)
        time.sleep(3.0)

        # Position 2
        self.send_joint_trajectory([-1.0, 0.7, -0.7, -0.8], 3.0)
        time.sleep(3.0)

        # Back to home
        self.send_joint_trajectory([0.0, 0.0, 0.0, 0.0], 2.0)
        self.get_logger().info("Demo sequence completed!")


def main(args=None):
    rclpy.init(args=args)
    controller = SmoreBotController()

    try:
        controller.run_demo()
    except Exception as e:
        controller.get_logger().error(f"Error in demo: {str(e)}")
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
