from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction, Shutdown, RegisterEventHandler,DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    map_name = LaunchConfiguration('map_name', default='my_map.yaml')
    params_file_name = LaunchConfiguration('params_file_name', default='param1.yaml')
    gazebo_map = LaunchConfiguration('gazebo_map',default='turtlebot3_world')
    goal_x = LaunchConfiguration('goal_x', default='0.6')
    goal_y = LaunchConfiguration('goal_y', default='0.6')
    goal_yaw = LaunchConfiguration('goal_yaw', default='0.0')
    # Konstrukcja pełnych ścieżek do mapy i pliku parametrów
    map_file = PathJoinSubstitution(['./src/turtlebot3_sim/maps', map_name])
    params_file = PathJoinSubstitution(['./src/turtlebot3_sim/paramy', params_file_name])
    return LaunchDescription([
        # Set environment variables
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle'),
        SetEnvironmentVariable('use_sim_time', 'True'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        DeclareLaunchArgument('map_name', default_value='my_map.yaml', description='Name of the map file'),
        DeclareLaunchArgument('params_file_name', default_value='param1.yaml', description='Name of the params file'),
        DeclareLaunchArgument('gazebo_map', default_value='turtlebot3_world', description='Name of the Gazebo world launch file'),
        DeclareLaunchArgument('goal_x', default_value='0.6', description='Goal X coordinate'),
        DeclareLaunchArgument('goal_y', default_value='0.6', description='Goal Y coordinate'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0', description='Goal yaw angle in radians'),
        # Launch Gazebo
        ExecuteProcess(
            cmd=['ros2', 'launch', 'turtlebot3_gazebo', [gazebo_map,'.launch.py']],
            output='screen',
            name='gazebo',
            on_exit=Shutdown()
        ),
        # Launch Navigation2 and RViz2 with increased delay
        TimerAction(
            period=10.0,  # Increased from 5.0 to 10.0 seconds
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py',
                         'use_sim_time:=True',
                         ['map:=',map_file],
                         ['params_file:=',params_file]],
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
                    parameters=[{'use_sim_time': True},
                    {'goal_x': goal_x},
                    {'goal_y': goal_y},
                    {'goal_yaw': goal_yaw}]
                )
            ]
        ),

        # # Handler to clean up after Gazebo exits
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=ExecuteProcess(
        #             cmd=['ros2', 'launch', 'turtlebot3_gazebo',  [gazebo_map,'.launch.py']]
        #         ),
        #         on_exit=[
        #             ExecuteProcess(
        #                 cmd=['killall', '-9', 'gzserver', 'gzclient', 'rviz2', 'ros2'],
        #                 output='screen'
        #             )
        #         ]
        #     )
        # ),

        # # Handler to clean up after Navigation2 exits
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=ExecuteProcess(
        #             cmd=['ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py', 
        #                  'use_sim_time:=True', 'map:=my_map.yaml']
        #         ),
        #         on_exit=[
        #             ExecuteProcess(
        #                 cmd=['killall', '-9', 'gzserver', 'gzclient', 'rviz2', 'ros2'],
        #                 output='screen'
        #             )
        #         ]
        #     )
        # )
    ])