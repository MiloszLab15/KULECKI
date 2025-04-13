#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import math
import signal
import os
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
import rosbag2_py
from rclpy.serialization import serialize_message

class TurtleBot3Sim(Node):
    def __init__(self):
        super().__init__('turtlebot3_sim_node')
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._amcl_pose_callback, amcl_pose_qos
        )
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = "map"
        self.current_pose = None
        self.initial_pose_received = False
        self.dodatek_x = 2.0
        self.dodatek_y = 0.5
        self.current_amcl_pose = None
        signal.signal(signal.SIGINT, self.signal_handler)

        # Initialize trajectory storage
        self.trajectory = []
        self.start_time = None

        # Initialize rosbag2 writer with unique bag file name
        timestamp = time.strftime("%Y%m%d_%H%M%S")  # e.g., 20250410_123456
        bag_dir = f'trajectory_bag_{timestamp}'
        storage_options = rosbag2_py.StorageOptions(
            uri=bag_dir,
            storage_id='sqlite3'
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        self.bag_writer = rosbag2_py.SequentialWriter()
        self.bag_writer.open(storage_options, converter_options)
        topic_info = rosbag2_py.TopicMetadata(
            name='/trajectory',
            type='geometry_msgs/msg/PoseStamped',
            serialization_format='cdr'
        )
        self.bag_writer.create_topic(topic_info)

        # Goal tracking variables
        self.goal_pose = None
        self.goal_reached = False
        self.goal_tolerance = 0.25

    # ... (rest of the class remains unchanged)

    def quaternion_to_yaw(self, qx, qy, qz, qw):
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_quaternion(self, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return (0.0, 0.0, qz, qw)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def _amcl_pose_callback(self, msg):
        if self.goal_reached:
            self.get_logger().info(f"DOTARLES:{self.goal_reached}")
            return

        self.initial_pose_received = True
        pose = msg.pose.pose
        self.current_amcl_pose = pose
        yaw = self.quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )
        self.get_logger().info(f"Current position (from RViz2/AMCL): x={pose.position.x}, "
                               f"y={pose.position.y}, yaw={yaw}")

        # Record trajectory to bag file
        if self.start_time is None:
            self.start_time = time.time()
        current_time = time.time() - self.start_time

        # Create PoseStamped message for trajectory
        traj_msg = PoseStamped()
        traj_msg.header.frame_id = "map"
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.pose.position.x = pose.position.x
        traj_msg.pose.position.y = pose.position.y
        traj_msg.pose.position.z = 0.0
        traj_msg.pose.orientation = pose.orientation

        # Write to bag file
        self.bag_writer.write(
            '/trajectory',
            serialize_message(traj_msg),
            self.get_clock().now().nanoseconds
        )

        # Store trajectory for plotting
        self.trajectory.append({
            'time': current_time,
            'x': pose.position.x,
            'y': pose.position.y,
            'yaw': yaw
        })

        # Check if goal is reached
        if self.goal_pose is not None:
            dx = pose.position.x - self.goal_pose.pose.position.x
            dy = pose.position.y - self.goal_pose.pose.position.y
            distance_to_goal = math.sqrt(dx**2 + dy**2)
            self.get_logger().info(f"DISTANCE:{distance_to_goal}")
            if distance_to_goal < self.goal_tolerance:
                self.get_logger().info("Goal reached!")
                self.goal_reached = True
                self.plot_trajectory()

    def plot_trajectory(self):
        """Plot the trajectory."""
        x = [point['x'] for point in self.trajectory]
        y = [point['y'] for point in self.trajectory]

        plt.figure(figsize=(8, 6))
        plt.plot(x, y, 'b-', label='Trajectory')
        plt.plot(x[0], y[0], 'go', label='Start')
        plt.plot(x[-1], y[-1], 'ro', label='End')
        if self.goal_pose is not None:
            plt.plot(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y,
                     'r*', label='Goal', markersize=15)
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('Robot Trajectory')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()

    def set_initial_pose_from_current(self):
        timeout = 10.0
        start_time = time.time()
        while self.current_pose is None and time.time() - start_time < timeout:
            self.get_logger().info("Waiting for odometry data...")
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.current_pose is None:
            self.get_logger().warn("No odometry data received after timeout, cannot set initial pose.")
            return False

        timeout = 10.0
        start_time = time.time()
        while self.count_subscribers('/initialpose') == 0 and time.time() - start_time < timeout:
            self.get_logger().info("Waiting for subscribers to /initialpose...")
            rclpy.spin_once(self, timeout_sec=0.5)
        
        if self.count_subscribers('/initialpose') == 0:
            self.get_logger().warn("No subscribers to /initialpose after timeout.")
            return False
        
        self.initial_pose.header.stamp = self.get_clock().now().to_msg()
        self.initial_pose.pose.position = self.current_pose.position
        self.initial_pose.pose.orientation = self.current_pose.orientation

        self.initial_pose_received = False
        self._set_initial_pose()
        self._wait_for_initial_pose()
        return True

    def _set_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.initial_pose.header.frame_id
        msg.header.stamp = self.initial_pose.header.stamp
        self.initial_pose.pose.position.x += self.dodatek_x
        self.initial_pose.pose.position.y += self.dodatek_y
        self.initial_pose.pose.position.z += 0.0
        msg.pose.pose = self.initial_pose.pose
        msg.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.01]

        self.get_logger().info("Publishing initial pose multiple times...")
        for _ in range(5):
            self.initial_pose_pub.publish(msg)
            time.sleep(0.5)
        self.get_logger().info("Initial pose published.")

    def _wait_for_initial_pose(self):
        max_wait_time = 10.0
        start_time = time.time()
        while not self.initial_pose_received and rclpy.ok():
            if time.time() - start_time > max_wait_time:
                self.get_logger().warn("Timeout reached while waiting for AMCL acknowledgment.")
                break
            self.get_logger().info("Waiting for AMCL to acknowledge initial pose...")
            rclpy.spin_once(self, timeout_sec=1.0)
        if self.initial_pose_received:
            self.get_logger().info("Initial pose acknowledged by AMCL!")
        else:
            self.get_logger().warn("AMCL did not acknowledge the initial pose in time.")

    def set_goal_pose(self, x, y, yaw):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.position.z = 0.0
        qx, qy, qz, qw = self.yaw_to_quaternion(yaw)
        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw
        self.goal_pose = goal_pose
        self.goal_pose_pub.publish(goal_pose)
        self.get_logger().info(f"Set goal: x={x}, y={y}, yaw={yaw}")

    def signal_handler(self, sig, frame):
        self.get_logger().info("Shutting down Gazebo, RViz2, and ROS 2 processes...")
        # Close the bag writer
        del self.bag_writer
        os.system("killall -9 gzserver gzclient rviz2 ros2")
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot3Sim()

    try:
        timeout = 10.0
        start_time = time.time()
        while rclpy.ok() and node.current_pose is None and time.time() - start_time < timeout:
            rclpy.spin_once(node)
            time.sleep(0.1)

        if node.current_pose is None:
            node.get_logger().error("Failed to receive odometry data, exiting.")
            node.signal_handler(None, None)
            return

        if node.set_initial_pose_from_current():
            time.sleep(5.0)
            node.set_goal_pose(0.6 + node.dodatek_x, 0.6 + node.dodatek_y, 0.0)
        
        while rclpy.ok() and not node.goal_reached:
            rclpy.spin_once(node)
            time.sleep(0.01)

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by Ctrl+C, shutting down...")
        node.signal_handler(None, None)
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        if rclpy.ok():
            node.signal_handler(None, None)

if __name__ == '__main__':
    main()