from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
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

    return LaunchDescription([
        use_gui,
        jsp_gui,
        rsp,
        rviz
    ])