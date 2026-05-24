import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    sim_headless = LaunchConfiguration('sim_headless', default='true')
    sim_software_rendering = LaunchConfiguration('sim_software_rendering', default='true')
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'))

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nav_map = get_package_share_directory('nav_map')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    gz_sim_resource_path = os.path.join(pkg_tb3_gazebo, 'models')
    world_file = os.path.join(pkg_nav_map, 'worlds', 'complex_maze.sdf')

    ld.add_action(SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_sim_resource_path,
    ))

    ld.add_action(DeclareLaunchArgument(
        'sim_headless',
        default_value='true',
        description='Run Gazebo without the GUI when true.'))

    ld.add_action(DeclareLaunchArgument(
        'sim_software_rendering',
        default_value='true',
        description='Force software rendering for Gazebo GUI stability when GUI is enabled.'))

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'headless': sim_headless,
            'use_software_rendering': sim_software_rendering,
        }.items(),
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        output='screen'
    )

    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Static Transform Publisher to fix Gazebo to URDF frame mismatch for laser scan
    tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0',
            '0',
            '0',
            '0',
            '0',
            '0',
            'base_scan',
            'turtlebot3_burger/base_scan/hls_lfcd_lds',
        ],
        output='screen'
    )

    ld.add_action(gz_sim)
    ld.add_action(bridge)
    ld.add_action(rsp)
    ld.add_action(tf_laser)
    return ld