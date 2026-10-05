import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.substitutions import Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# 1. Añadimos la importación que pide el error:
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg = get_package_share_directory('robotino_description')

    use_gui = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Use joint_state_publisher_gui to move joints'
    )

    urdf_file = PathJoinSubstitution([pkg, 'urdf', 'robotino_description.urdf.xacro']) 
    
    # 2. ENVOLVEMOS el comando en ParameterValue con value_type=str
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    jsp_gui = Node(
        condition=None,
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[],
        parameters=[{'rate': 30.0}],
        emulate_tty=True
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        # Aquí ahora lo lee como un texto sin problemas
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'default_view.rviz'])],
        output='screen'
    )
    
    # Enseñar a Gazebo dónde están los archivos STL
    set_model_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg, '..')
    )

    # Launch Gazebo Harmonic with an empty world
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': 'empty.sdf -r -v4'}.items()
    )
    
    # Spawn the robot in Gazebo Harmonic
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_robot'
        ],
        output='screen'
    )

    # Puente (Bridge) para conectar ROS 2 Jazzy con Gazebo Harmonic
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Comando de velocidad: ROS 2 -> Gazebo
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            
            # Odometría: Gazebo -> ROS 2
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            
            # Transformadas (TF): Gazebo -> ROS 2
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        output='screen'
    )

    return LaunchDescription([
        set_model_path,
        use_gui,
        jsp_gui,
        rsp,
        rviz,
        gazebo,
        spawn_robot,
        bridge
    ])