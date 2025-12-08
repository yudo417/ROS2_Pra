import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = 'a_description'
    pkg_share = get_package_share_directory(pkg_name)
    xacro_file = 'urdf/a.xacro'
    xacro_path = os.path.join(pkg_share, xacro_file)

    # 1. ここで強制的に環境変数をセットします！
    # 現在のパス設定に、このパッケージのパスを追記します
    gz_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '') + ':' + 
            os.path.join(pkg_share, '..') 
        ]
    )

    # 2. URDFの読み込み
    robot_description = {'robot_description': Command(['xacro ', xacro_path])}

    # 3. Ignition Gazebo の起動
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 4. ロボットの召喚 (Spawn)
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'my_robot',
                   '-z', '0.1'],
        output='screen'
    )

    # 5. RSP (ロボットの状態配信)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 6. Bridge (通信ブリッジ)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,  # <--- これを追加しました！
        ign_gazebo,
        spawn,
        node_robot_state_publisher,
        bridge
    ])