from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction, Shutdown, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Set environment variables
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle'),
        SetEnvironmentVariable('use_sim_time', 'True'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        
        # Launch Gazebo
        ExecuteProcess(
            cmd=['ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_world.launch.py'],
            output='screen',
            name='gazebo',
            on_exit=Shutdown()
        ),
        # # Launch Gazebo
        # ExecuteProcess(
        #     cmd=['ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_house.launch.py'],
        #     output='screen',
        #     name='gazebo',
        #     on_exit=Shutdown()
        # ),
        
        # Launch Navigation2 and RViz2 with increased delay
        TimerAction(
            period=10.0,  # Increased from 5.0 to 10.0 seconds
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py', 
                         'use_sim_time:=True', 'map:=my_map.yaml','params_file:=param1.yaml'],
                    output='screen',
                    name='navigation2',
                    on_exit=Shutdown()
                )
            ]
        ),
        
        # Launch custom node with additional delay
        TimerAction(
            period=15.0,  # Start custom node after Navigation2 is fully up
            actions=[
                Node(
                    package='turtlebot3_sim',
                    executable='start_turtlebot3',
                    name='turtlebot3_sim_node',
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                )
            ]
        ),

        # Handler to clean up after Gazebo exits
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ExecuteProcess(
                    cmd=['ros2', 'launch', 'turtlebot3_gazebo', 'turtleb3_world.launch.py']
                ),
                on_exit=[
                    ExecuteProcess(
                        cmd=['killall', '-9', 'gzserver', 'gzclient', 'rviz2', 'ros2'],
                        output='screen'
                    )
                ]
            )
        ),

        # Handler to clean up after Navigation2 exits
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ExecuteProcess(
                    cmd=['ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py', 
                         'use_sim_time:=True', 'map:=my_map.yaml']
                ),
                on_exit=[
                    ExecuteProcess(
                        cmd=['killall', '-9', 'gzserver', 'gzclient', 'rviz2', 'ros2'],
                        output='screen'
                    )
                ]
            )
        )
    ])