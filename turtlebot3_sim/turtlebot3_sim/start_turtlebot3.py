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
from rclpy.serialization import serialize_message
import numpy as np
from rcl_interfaces.msg import Log

class TurtleBot3Sim(Node):
    def __init__(self):
        super().__init__('turtlebot3_sim_node')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.log_sub = self.create_subscription(Log, '/rosout', self.log_callback, 10)
        self.navigator = BasicNavigator()
        self.plan_time = 0.0
        self.trajectory = []
        self.start_time = None
        self.current_pose = None
        self.dodatek_x = 2.1 #offsety
        self.dodatek_y = 0.6
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
    def log_callback(self, msg):
        # Capture any timing-related messages from planner_server and controller_server
        if msg.name in ["planner_server", "controller_server"]:
            # Log all messages for debugging
            self.get_logger().debug(f"Log from {msg.name}: {msg.msg}")
            # Try to extract planning times
            if "seconds" in msg.msg.lower() and any(keyword in msg.msg.lower() for keyword in ["plan", "planning", "control", "compute"]):
                try:
                    # Extract number before "seconds"
                    words = msg.msg.split()
                    for i, word in enumerate(words):
                        if "seconds" in word.lower():
                            time_str = words[i-1]
                            planning_time = float(time_str)
                            if msg.name == "planner_server":
                                self.planning_times['global'].append(planning_time)
                                self.get_logger().info(f"Global planning time: {planning_time} seconds")
                            else:
                                self.planning_times['local'].append(planning_time)
                                self.get_logger().info(f"Local planning time: {planning_time} seconds")
                            break
                except (IndexError, ValueError):
                    self.get_logger().warn(f"Could not parse time from log: {msg.msg}")
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        # Record trajectory to bag file
        if self.start_time is None:
            self.start_time = time.time()
        current_time = time.time() - self.start_time

        # Create PoseStamped message for trajectory
        traj_msg = PoseStamped()
        traj_msg.header.frame_id = "map"
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.pose = self.current_pose

        # Write to bag file
        self.bag_writer.write(
            '/trajectory',
            serialize_message(traj_msg),
            self.get_clock().now().nanoseconds
        )

        # Store trajectory for plotting
        yaw = self.quaternion_to_yaw(
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        )
        self.trajectory.append({
            'time': current_time,
            'x': self.current_pose.position.x+self.dodatek_x,
            'y': self.current_pose.position.y+self.dodatek_y,
            'yaw': yaw
        })


    #PLOTOWANIE TRAJEKTORII NA PODSATWIE WARTOŚCI Z SELF.TRAJECTORY
    def plot_trajectory(self):
        # funkcja pomocnicza - oblicza dlugosc sciezki
        def path_length(x, y):
            if not isinstance(x, list) or not isinstance(y, list):
                raise TypeError("ERROR: Arguments must be lists of type 'list'.")
            if x is not None and y is not None and len(x) > 1 and len(y) > 1:
                x = np.array(x)
                y = np.array(y)
                dx = np.diff(x)
                dy = np.diff(y)
                segment_lengths = np.sqrt(dx**2 + dy**2)
                path_length = np.sum(segment_lengths)
                return path_length
            else:
                print("ERROR!: Trajectory is empty!")
                return 0.0
            
        x = [point['x'] for point in self.trajectory]
        y = [point['y'] for point in self.trajectory]
        # licz dlugosc sceizki
        total_length = path_length(x, y)
        # pokaz cel

        # Get planning time (use first global planning time or handle empty list)
        planning_time = self.planning_times['local'][0] if self.planning_times['global'] else 0.0
        
        plt.figure(figsize=(8, 6))
        plt.plot(self.goal_x, self.goal_y,
                     'r*', label='Goal', markersize=15)
        plt.plot(x, y, 'b-', label='Trajectory')
        plt.plot(x[0], y[0], 'go', label='Start')
        plt.plot(x[-1], y[-1], 'ro', label='End')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        # plt.title('Robot Trajectory')
        # wyswietl dlugosc sciezki i czas planowania
        plt.title(f'Robot Trajectory\nPath length: {total_length:.2f} m \n Time to reach goal: {self.plan_time:.2f}\n Planning time: {planning_time:.2f} s')
        plt.grid(True)
        plt.axis('equal')
        plt.show(block=True)
        #
        #plt.savefig("trajectory_plot.png")
        self.get_logger().info("Zapisano wykres jako trajectory_plot.png")
        print(">>> Wywołano plot_trajectory")
        input(">>> Wciśnij Enter, żeby zakończyć program po obejrzeniu wykresu...")
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

        self.get_logger().info(f"Navigating to goal: x={x}, y={y}, yaw={yaw}")
        # start - poczatek planowania
        start_plan_time = time.time()
        self.navigator.goToPose(goal_pose)
        #TUTAJ SPRAWDZAMY CZY NASZ ROBOCIK SIĘ NIE ŚCIĄŁ GDZIEŚ
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                try:
                    #POWYŻEJ 5MIN
                    if feedback.estimated_time_remaining.sec > 600:  # 10 minutes timeout
                        self.get_logger().warn("Navigation taking too long, canceling task.")
                        self.navigator.cancelTask()
                        break
                except AttributeError as e:
                    self.get_logger().warn(f"Feedback attribute error: {e}. Continuing without timeout check.")
            rclpy.spin_once(self, timeout_sec=0.01)
        #stop - koniec planowania - oblicz czas
        self.plan_time = time.time() - start_plan_time
       #SPRAWDZAMY REZULTAT W RAZIE OSIĄGNIĘCIA CELU PLOTUJEMY TRAJEKTORIE
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
            print("Plan time: ", self.plan_time)
            self.plot_trajectory()
        elif result == TaskResult.CANCELED:
            self.get_logger().warn("Goal was canceled!")
        elif result == TaskResult.FAILED:
            self.get_logger().error("Goal failed!")
        return result == TaskResult.SUCCEEDED
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

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot3Sim()

    try:
        if node.set_initial_pose_from_current():
            time.sleep(2.0)  # Allow Nav2 to stabilize
            # Use parameters for goal pose
            node.set_goal_pose(
                node.goal_x,
                node.goal_y,
                node.goal_yaw
            )
        
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

if __name__ == '__main__':
    main()