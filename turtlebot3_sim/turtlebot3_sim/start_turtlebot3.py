#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import math
import signal
import os
import shutil
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rosbag2_py
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage
from rclpy.serialization import serialize_message
import uuid
class TurtleBot3Sim(Node):
    def __init__(self):
        super().__init__('turtlebot3_sim_node')        
        self.get_logger().info("Initializing turtlebot3_sim_node")
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.odom_sub = self.create_subscription(Odometry,'/odom', self.odom_callback, 10)
        self.navigator = BasicNavigator()
        self.plan_time = 0.0
        self.trajectory = []
        self.start_time = None
        self.current_pose = None
        self.dodatek_x = 2.0 #offsety
        self.dodatek_y = 0.5
        self.navigation_start_time = None  # To track navigation start
        self.navigation_time = 0.0  # To store total navigation time
        self.total_cost = 0.0
        self.declare_parameter('goal_x')
        self.declare_parameter('goal_y')
        self.declare_parameter('goal_yaw')
        self.goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        self.goal_y = self.get_parameter('goal_y').get_parameter_value().double_value
        self.goal_yaw = self.get_parameter('goal_yaw').get_parameter_value().double_value
        timestamp = time.strftime("%Y%m%d_%H%M%S")#żeby mieć unikalną nazwę rosbag bo się wysypuje inaczej
        bag_dir = f'./src/turtlebot3_sim/trajectory/trajectory_bag_{self.goal_x}_{self.goal_y}'
        if os.path.exists(bag_dir) and os.path.isdir(bag_dir):
            # Usunięcie folderu i jego zawartości
            shutil.rmtree(bag_dir)
        storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3')#format dla ros2
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        self.bag_writer = rosbag2_py.SequentialWriter()#rozpoczynanie zapisywania
        self.bag_writer.open(storage_options, converter_options)
        topic_info = rosbag2_py.TopicMetadata(#dajemy tu nasze topici wraz z typem
            name='/trajectory',
            type='geometry_msgs/msg/PoseStamped',
            serialization_format='cdr'
        )
        self.bag_writer.create_topic(topic_info)
        self.planning_times = {'global': [], 'local': []}
        signal.signal(signal.SIGINT, self.signal_handler)
    #KWATERNIONY NA KĄTY
    def quaternion_to_yaw(self, qx, qy, qz, qw):
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)
    #KĄTY NA KWATERNIONY
    def yaw_to_quaternion(self, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return (0.0, 0.0, qz, qw)
    #ODOMETRIA ZWRACANIE POZYCJI ORAZ ZAPIS TRAJEKTORII DO ROSBAG
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        

    #PLOTOWANIE TRAJEKTORII NA PODSATWIE WARTOŚCI Z SELF.TRAJECTORY
    def plot_trajectory(self):
        def path_length(self, x, y):
            if not isinstance(x, list) or not isinstance(y, list):
                raise TypeError("Arguments must be lists.")
            if len(x) < 2 or len(y) < 2:
                self.get_logger().error("Trajectory is empty or too short!")
                return 0.0
            x = np.array(x)
            y = np.array(y)
            cost_map = self.navigator.getGlobalCostmap()
            width, height = cost_map.metadata.size_x, cost_map.metadata.size_y
            resolution = cost_map.metadata.resolution
            origin_x, origin_y = cost_map.metadata.origin.position.x, cost_map.metadata.origin.position.y

            self.total_cost = 0.0
            for i in range(len(x)):
                # Convert world coordinates to grid indices
                grid_x = int((x[i] - origin_x) / resolution)
                grid_y = int((y[i] - origin_y) / resolution)
                # Check if indices are within costmap bounds
                if 0 <= grid_x < width and 0 <= grid_y < height:
                    index = grid_y * width + grid_x  # Row-major indexing
                    self.total_cost += cost_map.data[index]
                else:
                    self.get_logger().warn(f"Point ({x[i]}, {y[i]}) is outside costmap bounds!")

            dx = np.diff(x)
            dy = np.diff(y)
            segment_lengths = np.sqrt(dx**2 + dy**2)
            return np.sum(segment_lengths)

        x = [point['x'] for point in self.trajectory]
        y = [point['y'] for point in self.trajectory]
        
        total_length = path_length(self,x, y)
        plt.figure(figsize=(12, 8))
        plt.plot(self.goal_x, self.goal_y, 'r*', label='Goal', markersize=15)
        plt.plot(x, y, 'b-', label='Trajectory')
        plt.plot(x[0], y[0], 'go', label='Start')
        plt.plot(x[-1], y[-1], 'ro', label='End')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title(
            f'Robot Trajectory\n'
            f'Path length: {total_length:.2f} m\n'
            f'Path cost: {self.total_cost:.2f} (cost-based)\n'
            f'Planning time: {self.plan_time:.2f} s\n'
            f'Navigation time: {self.navigation_time:.2f} s'
        )
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.savefig('robot_trajectory.png')
        plt.show()  # Keep show for local testing; comment out if running headless
        self.get_logger().info("Zapisano wykres trajektorii do robot_trajectory.png.")
    #USTAWIANIE POZYCJI POCZĄTKOWEJ
    def set_initial_pose_from_current(self):
        timeout = 10.0
        start_time = time.time()
        #SPRAWDZANIE CZY POZYCJA POCZĄTKOWE JEST PUSTA  I CZEKANIE 10 SEC TAK DLA PEWNOŚCI POMIARÓW Z ODOMETRII
        while self.current_pose is None and time.time() - start_time < timeout:
            self.get_logger().info("Waiting for odometry data...")
            rclpy.spin_once(self, timeout_sec=0.1)
        #JEŻELI POMIMO TEGO NIE MAMY ODOMETRII
        if self.current_pose is None:
            self.get_logger().error("No odometry data received after timeout.")
            return False
        #USTAWIANIE INITIAL NA PODSTAWIE ODOMETRII I RĘCZNEGO PRZESUNIĘCIA
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.current_pose.position.x + self.dodatek_x
        initial_pose.pose.position.y = self.current_pose.position.y + self.dodatek_y
        initial_pose.pose.position.z = 0.0
        initial_pose.pose.orientation = self.current_pose.orientation
        #PRZEKAZYWANIE INITIAL POSE I CZEKANIE NA AKTYWACJE AMCL
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Initial pose set and Nav2 is active.")
        return True
    #USTAWIANIE GOAL
    def measure_plan_time(self, initial_pose: PoseStamped, goal_pose: PoseStamped, planner_id: str = ''):
        start_time = time.time()
        path = self.navigator._getPathImpl(initial_pose, goal_pose, planner_id, use_start=True)
        plan_time = time.time() - start_time

        if path is not None:
            self.get_logger().info(f"Planning succeeded in {plan_time:.4f} seconds.")
        else:
            self.get_logger().error("Planning failed.")

        return path, plan_time
    def execute_navigation(self, goal_pose: PoseStamped):
        self.navigation_start_time = time.time()  # Record start time
        self.navigator.goToPose(goal_pose)
        # Create PoseStamped message for trajectory
        traj_msg = PoseStamped()
        traj_msg.header.frame_id = "map"
        



        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            # self.get_logger().info(f"Pozycja {feedback.current_pose}")
            traj_msg.header.stamp = self.get_clock().now().to_msg()
            traj_msg.pose.position.x = feedback.current_pose.pose.position.x
            traj_msg.pose.position.y = feedback.current_pose.pose.position.y
            traj_msg.pose.position.z = 0.0
            traj_msg.pose.orientation = feedback.current_pose.pose.orientation
            # Write to bag file
            self.bag_writer.write(
                '/trajectory',
                serialize_message(traj_msg),
                self.get_clock().now().nanoseconds
            )
            self.trajectory.append({
                'x': feedback.current_pose.pose.position.x,
                'y': feedback.current_pose.pose.position.y})
            if feedback:
                try:
                    if feedback.estimated_time_remaining.sec > 600:
                        self.get_logger().warn("Navigation taking too long, canceling task.")
                        self.navigator.cancelTask()
                        break
                except AttributeError:
                    pass
            rclpy.spin_once(self, timeout_sec=0.01)

        self.navigation_time = time.time() - self.navigation_start_time  # Calculate total navigation time
        for _ in range(100):  # Spin for ~1 second
            rclpy.spin_once(self, timeout_sec=0.01)
        result = self.navigator.getResult()
        return result
    #TUTAJ OBSŁUGA CTRL + C ŻEBY W TEORII ZABIJAŁO MI PROCESY Z GAZEBO RVIZ2 I ROS2 NIE ZAWSZE DZIALA
    def signal_handler(self, sig, frame):
        self.get_logger().info("Shutting down Gazebo, RViz2, and ROS 2 processes...")
        del self.bag_writer
        os.system("killall -9 gzserver gzclient rviz2 ros2")
        self.navigator.cancelTask()
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        exit(0)
    def create_goal_pose(self):
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = self.goal_x
            goal_pose.pose.position.y = self.goal_y
            qx, qy, qz, qw = self.yaw_to_quaternion(self.goal_yaw)
            goal_pose.pose.orientation.x = qx
            goal_pose.pose.orientation.y = qy
            goal_pose.pose.orientation.z = qz
            goal_pose.pose.orientation.w = qw
            return goal_pose
def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot3Sim()

    try:
        if node.set_initial_pose_from_current():
            time.sleep(2.0)

            initial_pose = PoseStamped()
            initial_pose.header.frame_id = "map"
            initial_pose.header.stamp = node.get_clock().now().to_msg()
            initial_pose.pose.position.x = node.current_pose.position.x+node.dodatek_x
            initial_pose.pose.position.y = node.current_pose.position.y+node.dodatek_y
            initial_pose.pose.orientation = node.current_pose.orientation

            goal_pose = node.create_goal_pose()

            path, plan_time = node.measure_plan_time(initial_pose, goal_pose)
            node.plan_time = plan_time
            print(f">>> Czas planowania ścieżki: {plan_time:.4f} sekund.")

            if path is not None:
                result = node.execute_navigation(goal_pose)
                if result == TaskResult.SUCCEEDED:
                    
                    node.get_logger().info("Goal succeeded!")
                    time.sleep(2.0)
                    node.plot_trajectory()
                elif result == TaskResult.CANCELED:
                    node.get_logger().warn("Goal was canceled!")
                elif result == TaskResult.FAILED:
                    node.get_logger().error("Goal failed!")
            else:
                node.get_logger().error("Path planning failed - navigation skipped.")

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by Ctrl+C, shutting down...")
        node.signal_handler(None, None)
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
        node.signal_handler(None, None)
    finally:
        node.signal_handler(None, None)

# if __name__ == '__main__':
#     main()