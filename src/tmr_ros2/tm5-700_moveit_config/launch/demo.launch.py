import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. MoveIt設定の構築
    moveit_config = (
        MoveItConfigsBuilder("tm5-700")
        .robot_description(file_path="config/tm5-700.urdf.xacro")
        .robot_description_semantic(file_path="config/tm5-700.srdf")
        .planning_pipelines(pipelines=["ompl"])
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # 2. コントローラーファイルのパス取得と読み込み (NameErrorの修正)
    controllers_file = os.path.join(
        get_package_share_directory("tm5-700_moveit_config"),
        "config",
        "moveit_controllers.yaml",
    )
    with open(controllers_file, 'r') as f:
        controllers_yaml = yaml.safe_load(f)

    # 3. move_group ノードの設定
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            controllers_file,
            controllers_yaml,
            {
                "moveit_manage_controllers": True,
                "trajectory_execution.execution_duration_monitoring": False,
                "trajectory_execution.publish_joint_external_point": True, # トピックへの配信を有効化
            }
        ],
    )
    
    # 4. RViz2 ノード
    rviz_config_file = os.path.join(
        get_package_share_directory("tm5-700_moveit_config"),
        "config",
        "moveit.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    # 5. Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )
    
    # 6. Static TF (virtual_joint用)
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base"],
    )

    return LaunchDescription([
        static_tf_node,
        robot_state_publisher,
        run_move_group_node,
        rviz_node,
    ])