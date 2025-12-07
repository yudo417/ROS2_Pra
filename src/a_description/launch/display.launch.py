import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # パッケージ名とxacroファイルのパス設定
    pkg_name = 'a_description'
    xacro_file = 'urdf/a.xacro'
    
    # インストールされたディレクトリからファイルのパスを取得
    pkg_share = get_package_share_directory(pkg_name)
    xacro_path = os.path.join(pkg_share, xacro_file)

    # ロボット記述（URDF）をxacroコマンドで生成する設定
    robot_description = {'robot_description': Command(['xacro ', xacro_path])}

    return LaunchDescription([
        # 1. 関節をGUIで動かすノード
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        # 2. ロボットの状態（TF）を配信するノード
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description]
        ),
        # 3. RViz2（可視化ツール）
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])