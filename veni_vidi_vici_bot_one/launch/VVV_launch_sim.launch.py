
import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():

    #----- Launch arguments -----

    activate_slam_arg = DeclareLaunchArgument('activate_slam', default_value='false', description='Flag to activate SLAM')
    activate_nav_arg  = DeclareLaunchArgument('activate_nav' , default_value='false', description='Flag to activate Nav2')
    activate_loc_arg  = DeclareLaunchArgument('activate_loc' , default_value='false', description='Flag to activate localisation')
    activate_cam_arg  = DeclareLaunchArgument('activate_cam' , default_value='false', description='Flag to activate camera')
    activate_sm_arg   = DeclareLaunchArgument('activate_sm'  , default_value='false', description='Flag to activate state machine')
    activate_sim_arg  = DeclareLaunchArgument('activate_sim'  , default_value='false', description='Flag to activate simulation')


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='veni_vidi_vici_bot_one' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )
 
    #----- Run command hierarchy node -----

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )
    

    #----- Launch lidar -----

    rplidar = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rplidar.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    #----- 

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    #----- Launch all the controllers

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    #----- Launch camera -----

    # yolov6_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('oakd_cam'), 'launch', 'yolov6_publisher.launch.py')]),
    #     condition=IfCondition(LaunchConfiguration('activate_cam'))
    # )   

    #----- Launch SLAM toolbox -----

    slam_toolbox_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={
            'params_file': os.path.join(get_package_share_directory(package_name), 'config', 'mapper_params_online_async.yaml'),
            'use_sim_time': 'false'
        }.items(),
        condition=IfCondition(LaunchConfiguration('activate_slam'))
    )

    delayed_slam_launch = TimerAction(
        period=5.0, 
        actions=[slam_toolbox_launch_description]
    )

    #----- Launch localization -----

    loc_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')
        ]),
        launch_arguments={
            'params_file': os.path.join(get_package_share_directory(package_name), 'config', 'nav2_params.yaml'),
            'map':os.path.join(get_package_share_directory(package_name), 'maps', 'my_map_save.yaml'),
            'use_sim_time': 'false'
        }.items(),
        condition=IfCondition(LaunchConfiguration('activate_loc'))
    )

    delayed_loc_launch = TimerAction(
        period=10.0, 
        actions=[loc_launch_description]
    )    

    #----- Launch navigation -----

    nav_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'params_file': os.path.join(get_package_share_directory(package_name), 'config', 'nav2_params.yaml'),
            'use_sim_time': 'false',
            'map_subscribe_transient_local': 'true',
            'log_level': 'error'
        }.items(),
        condition=IfCondition(LaunchConfiguration('activate_nav'))
    )

    delayed_nav_launch = TimerAction(
        period=10.0, 
        actions=[nav_launch_description]
    )   

    image_node = Node(
        package=package_name,
        executable='cam_process',
        name='cam_process',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('activate_sim'))
    )

    duplo_node = Node(
        package=package_name,
        executable='detect_duplo',
        name='detect_duplo',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('activate_sim'))
    )

    ball_pos_node = Node(
        package=package_name,
        executable='detect_ball_3d',
        name='detect_ball_3d',
        # parameters=[params_file],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('activate_sim'))
    )

    delayed_image_node = TimerAction(
        period=15.0, 
        actions=[image_node]
    )

    delayed_duplo_node = TimerAction(
        period=15.0, 
        actions=[duplo_node]
    )

    delayed_ball_pos_node = TimerAction(
        period=15.0, 
        actions=[ball_pos_node]
    )

    #----- Launch state machine -----

    sm_node = Node(
        package=package_name,
        executable='VVV_sm_node',
        name='VVV_sm_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('activate_sm'))
    )

    delayed_sm_node = TimerAction(
        period=15.0, 
        actions=[sm_node]
    )
    #----- Launch them all!

    return LaunchDescription([
        rsp,
        twist_mux,
        rplidar,

        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,

#        yolov6_launch,

        activate_loc_arg,
        activate_nav_arg,
        activate_slam_arg,
        activate_cam_arg,
        activate_sm_arg,
        activate_sim_arg,

        delayed_slam_launch,
        delayed_loc_launch,
        # delayed_nav_launch,
        delayed_sm_node,

        delayed_image_node,
        delayed_duplo_node,
        # delayed_ball_pos_node,

    ])