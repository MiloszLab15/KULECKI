#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import math
import signal
import os
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rosbag2_py
from rclpy.serialization import serialize_message

class TurtleBot3Sim(Node):
    def __init__(self):
        super().__init__('turtlebot3_sim_node')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        

        self.navigator = BasicNavigator()
        
        self.trajectory = []
        self.start_time = None
        self.current_pose = None
        self.dodatek_x = 2.1 #offsety
        self.dodatek_y = 0.6

        timestamp = time.strftime("%Y%m%d_%H%M%S")#żeby mieć unikalną nazwę rosbag bo się wysypuje inaczej
        bag_dir = f'./src/turtlebot3_sim/trajectory/trajectory_bag_{timestamp}'
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
            'x': self.current_pose.position.x,
            'y': self.current_pose.position.y,
            'yaw': yaw
        })
    #PLOTOWANIE TRAJEKTORII NA PODSATWIE WARTOŚCI Z SELF.TRAJECTORY
    def plot_trajectory(self):
        x = [point['x'] for point in self.trajectory]
        y = [point['y'] for point in self.trajectory]

        plt.figure(figsize=(8, 6))
        plt.plot(x, y, 'b-', label='Trajectory')
        plt.plot(x[0], y[0], 'go', label='Start')
        plt.plot(x[-1], y[-1], 'ro', label='End')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('Robot Trajectory')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()
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

       #SPRAWDZAMY REZULTAT W RAZIE OSIĄGNIĘCIA CELU PLOTUJEMY TRAJEKTORIE
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
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
            node.set_goal_pose(0.4 + node.dodatek_x, 0.6 + node.dodatek_y, 0.0)
        
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