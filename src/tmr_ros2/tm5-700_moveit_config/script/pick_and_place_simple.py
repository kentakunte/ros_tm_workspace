#!/usr/bin/env python3
"""
TM5-700 シンプルなピック＆プレースデモ
MoveIt Commanderを使用した簡易版
"""

import rclpy
from rclpy.node import Node
import sys
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
import math
import time


class SimplePickAndPlace(Node):
    def __init__(self):
        super().__init__('simple_pick_and_place')
        
        # MoveIt Commanderの初期化
        moveit_commander.roscpp_initialize(sys.argv)
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("tmr_arm")
        
        # 計画パラメータ設定
        self.move_group.set_planning_time(5.0)
        self.move_group.set_num_planning_attempts(10)
        self.move_group.set_max_velocity_scaling_factor(0.5)
        self.move_group.set_max_acceleration_scaling_factor(0.5)
        
        self.get_logger().info(f'リファレンスフレーム: {self.move_group.get_planning_frame()}')
        self.get_logger().info(f'エンドエフェクタ: {self.move_group.get_end_effector_link()}')
        
        # Planning Sceneパブリッシャー
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        
        time.sleep(1.0)  # シーン初期化待機

    def add_box(self, name, position, size=(0.05, 0.05, 0.05)):
        """ボックスをシーンに追加"""
        box_pose = PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position.x = position[0]
        box_pose.pose.position.y = position[1]
        box_pose.pose.position.z = position[2]
        box_pose.pose.orientation.w = 1.0
        
        self.scene.add_box(name, box_pose, size=size)
        self.get_logger().info(f'ボックス追加: {name} at ({position[0]}, {position[1]}, {position[2]})')
        time.sleep(0.5)

    def remove_box(self, name):
        """ボックスを削除"""
        self.scene.remove_world_object(name)
        self.get_logger().info(f'ボックス削除: {name}')
        time.sleep(0.5)

    def attach_box(self, name):
        """ボックスをアタッチ"""
        eef_link = self.move_group.get_end_effector_link()
        touch_links = ["link_5", "link_6", "flange"]
        self.scene.attach_box(eef_link, name, touch_links=touch_links)
        self.get_logger().info(f'ボックスアタッチ: {name}')
        time.sleep(0.5)

    def detach_box(self, name):
        """ボックスをデタッチ"""
        eef_link = self.move_group.get_end_effector_link()
        self.scene.remove_attached_object(eef_link, name=name)
        self.get_logger().info(f'ボックスデタッチ: {name}')
        time.sleep(0.5)

    def go_to_joint_state(self, joint_goal):
        """関節角度で移動"""
        self.get_logger().info('関節目標へ移動中...')
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        return True

    def go_to_pose_goal(self, pose_goal):
        """姿勢目標へ移動"""
        self.get_logger().info(f'姿勢目標へ移動: ({pose_goal.position.x:.2f}, {pose_goal.position.y:.2f}, {pose_goal.position.z:.2f})')
        self.move_group.set_pose_target(pose_goal)
        
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        return success

    def create_pose(self, x, y, z, roll=0.0, pitch=math.pi, yaw=0.0):
        """姿勢を作成（デフォルトは下向き）"""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        
        # オイラー角からクォータニオン
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        pose.orientation.w = cr * cp * cy + sr * sp * sy
        pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose.orientation.z = cr * cp * sy - sr * sp * cy
        
        return pose

    def execute_demo(self):
        """デモ実行"""
        box_name = "target_box"
        
        # 位置定義
        point_a = (0.4, 0.2, 0.1)    # A地点
        point_b = (0.4, -0.2, 0.1)   # B地点
        approach_height = 0.15       # アプローチ高さ
        grip_height = 0.08          # グリップ高さ
        
        try:
            # 1. ホームポジション
            self.get_logger().info('=== ステップ1: ホームポジション ===')
            home_joints = [0, 0, 0, 0, 0, 0]
            self.go_to_joint_state(home_joints)
            time.sleep(1.0)
            
            # 2. ボックス追加
            self.get_logger().info('=== ステップ2: ボックス追加 ===')
            self.add_box(box_name, point_a, size=(0.05, 0.05, 0.05))
            time.sleep(1.0)
            
            # 3. A地点上方へ
            self.get_logger().info('=== ステップ3: A地点上方へアプローチ ===')
            approach_a = self.create_pose(
                point_a[0], point_a[1], point_a[2] + approach_height
            )
            if not self.go_to_pose_goal(approach_a):
                self.get_logger().error('アプローチ失敗')
                return False
            time.sleep(1.0)
            
            # 4. 降下してグリップ
            self.get_logger().info('=== ステップ4: 降下してグリップ ===')
            grip_a = self.create_pose(
                point_a[0], point_a[1], point_a[2] + grip_height
            )
            if not self.go_to_pose_goal(grip_a):
                self.get_logger().error('グリップ位置への移動失敗')
                return False
            
            # ボックスをアタッチ（グリップ）
            self.attach_box(box_name)
            time.sleep(1.0)
            
            # 5. 持ち上げ
            self.get_logger().info('=== ステップ5: 持ち上げ ===')
            if not self.go_to_pose_goal(approach_a):
                self.get_logger().error('持ち上げ失敗')
                return False
            time.sleep(1.0)
            
            # 6. B地点上方へ移動
            self.get_logger().info('=== ステップ6: B地点上方へ移動 ===')
            approach_b = self.create_pose(
                point_b[0], point_b[1], point_b[2] + approach_height
            )
            if not self.go_to_pose_goal(approach_b):
                self.get_logger().error('B地点への移動失敗')
                return False
            time.sleep(1.0)
            
            # 7. 降下して配置
            self.get_logger().info('=== ステップ7: 降下して配置 ===')
            place_b = self.create_pose(
                point_b[0], point_b[1], point_b[2] + grip_height
            )
            if not self.go_to_pose_goal(place_b):
                self.get_logger().error('配置位置への移動失敗')
                return False
            
            # ボックスをデタッチ（解放）
            self.detach_box(box_name)
            # B地点に再配置
            self.remove_box(box_name)
            self.add_box(box_name, point_b, size=(0.05, 0.05, 0.05))
            time.sleep(1.0)
            
            # 8. 上昇
            self.get_logger().info('=== ステップ8: 上昇 ===')
            if not self.go_to_pose_goal(approach_b):
                self.get_logger().error('上昇失敗')
                return False
            time.sleep(1.0)
            
            # 9. ホームに戻る
            self.get_logger().info('=== ステップ9: ホームに戻る ===')
            self.go_to_joint_state(home_joints)
            time.sleep(1.0)
            
            self.get_logger().info('=== デモ完了！ ===')
            return True
            
        except Exception as e:
            self.get_logger().error(f'エラー: {str(e)}')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    node = SimplePickAndPlace()
    
    try:
        success = node.execute_demo()
        
        if success:
            node.get_logger().info('✓ ピック＆プレース成功！')
        else:
            node.get_logger().error('✗ ピック＆プレース失敗')
        
        # 結果を確認するため少し待機
        time.sleep(3.0)
        
    except KeyboardInterrupt:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
