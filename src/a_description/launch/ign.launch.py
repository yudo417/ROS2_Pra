import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = 'a_description'
    xacro_file = 'urdf/a.xacro'
    
    pkg_share = get_package_share_directory(pkg_name)
    xacro_path = os.path.join(pkg_share, xacro_file)

    # 1. URDFの読み込み
    robot_description = {'robot_description': Command(['xacro ', xacro_path])}

    # 2. Ignition Gazebo の起動 (空の世界: empty.sdf)
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 3. ロボットの召喚 (Spawn)
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'my_robot',
                   '-z', '0.1'], # 地面に埋まらないように少し浮かせる
        output='screen'
    )

    # 4. Robot State Publisher (TF配信)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 5. ブリッジ (ROS 2 <-> Ignition の通信)
    # これがないとIgnitionの情報がROSに来ません
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        ign_gazebo,
        spawn,
        node_robot_state_publisher,
        bridge
    ])