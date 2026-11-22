import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.substitutions import Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg = get_package_share_directory('robotino_description')

    use_gui = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Use joint_state_publisher_gui to move joints'
    )

    urdf_file = PathJoinSubstitution([pkg, 'urdf', 'robotino_description.urdf.xacro']) 

    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True  
        }],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'default_view.rviz'])],
        parameters=[{'use_sim_time': True}], 
        output='screen'
    )

    set_model_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg, '..')
    )

    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': 'empty.sdf -r -v4'}.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_robot'
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    wheel_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'wheel_velocity_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    return LaunchDescription([
        set_model_path,
        use_gui,
        rsp,
        rviz,
        joint_state_broadcaster_spawner,
        wheel_velocity_controller_spawner,
        gazebo,
        spawn_robot,
        bridge
    ])