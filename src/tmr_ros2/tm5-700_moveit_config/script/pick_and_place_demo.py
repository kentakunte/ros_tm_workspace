#!/usr/bin/env python3
"""
TM5-700 ピック＆プレースデモ
A地点のボックスをB地点に運ぶ動作を可視化
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
import math


class PickAndPlaceDemo(Node):
    def __init__(self):
        super().__init__('pick_and_place_demo')
        
        # MoveItPyの初期化
        self.get_logger().info('MoveItPyを初期化中...')
        self.moveit = MoveItPy(node_name="pick_and_place_demo")
        self.tmr_arm = self.moveit.get_planning_component("tmr_arm")
        self.planning_scene_monitor = self.moveit.get_planning_scene_monitor()
        
        # 位置定義（ワールド座標系）
        self.point_a = {'x': 0.4, 'y': 0.2, 'z': 0.2}   # A地点（ボックス初期位置）
        self.point_b = {'x': 0.4, 'y': -0.2, 'z': 0.2}  # B地点（目標位置）
        self.home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # グリップ高さオフセット
        self.approach_offset = 0.15  # アプローチ高さ（ボックス上方）
        self.grip_offset = 0.05      # グリップ高さ（ボックス把持位置）
        
        self.get_logger().info('初期化完了')

    def add_box_to_scene(self, name, position, size=(0.05, 0.05, 0.05)):
        """
        シーンにボックスを追加
        
        Args:
            name: ボックス名
            position: 位置 {'x': float, 'y': float, 'z': float}
            size: サイズ (x, y, z) デフォルト5cm立方体
        """
        with self.planning_scene_monitor.read_write() as scene:
            collision_object = CollisionObject()
            collision_object.header.frame_id = "world"
            collision_object.id = name
            
            # ボックス形状定義
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = list(size)
            
            # 位置定義
            box_pose = Pose()
            box_pose.position.x = position['x']
            box_pose.position.y = position['y']
            box_pose.position.z = position['z']
            box_pose.orientation.w = 1.0
            
            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(box_pose)
            collision_object.operation = CollisionObject.ADD
            
            scene.apply_collision_object(collision_object)
            
        self.get_logger().info(f'ボックス "{name}" を位置 ({position["x"]}, {position["y"]}, {position["z"]}) に追加')

    def remove_object_from_scene(self, name):
        """シーンからオブジェクトを削除"""
        with self.planning_scene_monitor.read_write() as scene:
            scene.remove_object(name)
        self.get_logger().info(f'オブジェクト "{name}" を削除')

    def attach_box(self, box_name):
        """ボックスをエンドエフェクタにアタッチ（把持をシミュレート）"""
        with self.planning_scene_monitor.read_write() as scene:
            scene.process_attached_collision_object(
                AttachedCollisionObject(
                    object=CollisionObject(
                        id=box_name,
                        operation=CollisionObject.REMOVE
                    ),
                    link_name="link_6",
                    touch_links=["link_5", "link_6"]
                )
            )
        self.get_logger().info(f'ボックス "{box_name}" をアタッチ')

    def detach_box(self, box_name):
        """ボックスをデタッチ（解放をシミュレート）"""
        with self.planning_scene_monitor.read_write() as scene:
            scene.process_attached_collision_object(
                AttachedCollisionObject(
                    object=CollisionObject(
                        id=box_name,
                        operation=CollisionObject.ADD
                    ),
                    link_name="link_6"
                )
            )
        self.get_logger().info(f'ボックス "{box_name}" をデタッチ')

    def move_to_home(self):
        """ホームポジションに移動"""
        self.get_logger().info('ホームポジションへ移動中...')
        self.tmr_arm.set_start_state_to_current_state()
        self.tmr_arm.set_goal_state(configuration_name="home")
        
        plan_result = self.tmr_arm.plan()
        if plan_result:
            self.tmr_arm.execute()
            return True
        else:
            self.get_logger().error('ホームポジションへの計画失敗')
            return False

    def move_to_pose(self, target_pose, description=""):
        """
        指定された姿勢に移動
        
        Args:
            target_pose: 目標姿勢 {'x': float, 'y': float, 'z': float, 'roll': float, 'pitch': float, 'yaw': float}
            description: 動作の説明
        """
        self.get_logger().info(f'{description}へ移動中...')
        
        # PoseStampedの作成
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "world"
        pose_stamped.pose.position.x = target_pose['x']
        pose_stamped.pose.position.y = target_pose['y']
        pose_stamped.pose.position.z = target_pose['z']
        
        # オイラー角からクォータニオンに変換
        roll = target_pose.get('roll', 0.0)
        pitch = target_pose.get('pitch', math.pi)  # デフォルトは下向き
        yaw = target_pose.get('yaw', 0.0)
        
        # クォータニオン計算（簡易版）
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        pose_stamped.pose.orientation.w = cr * cp * cy + sr * sp * sy
        pose_stamped.pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose_stamped.pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose_stamped.pose.orientation.z = cr * cp * sy - sr * sp * cy
        
        # 動作計画
        self.tmr_arm.set_start_state_to_current_state()
        self.tmr_arm.set_goal_state(pose_stamped_msg=pose_stamped, pose_link="flange")
        
        plan_result = self.tmr_arm.plan()
        if plan_result:
            self.tmr_arm.execute()
            return True
        else:
            self.get_logger().error(f'{description}への動作計画失敗')
            return False

    def execute_pick_and_place(self):
        """ピック＆プレース動作の実行"""
        try:
            # 1. シーンセットアップ：ボックスをA地点に配置
            self.get_logger().info('=== シーンセットアップ ===')
            self.add_box_to_scene("target_box", self.point_a)
            rclpy.spin_once(self, timeout_sec=1.0)
            
            # 2. ホームポジションへ
            self.get_logger().info('=== ステップ1: ホームポジションへ移動 ===')
            if not self.move_to_home():
                return False
            rclpy.spin_once(self, timeout_sec=1.0)
            
            # 3. A地点上方へアプローチ
            self.get_logger().info('=== ステップ2: A地点上方へアプローチ ===')
            approach_a = {
                'x': self.point_a['x'],
                'y': self.point_a['y'],
                'z': self.point_a['z'] + self.approach_offset,
                'pitch': math.pi
            }
            if not self.move_to_pose(approach_a, "A地点上方"):
                return False
            rclpy.spin_once(self, timeout_sec=1.0)
            
            # 4. 降下してボックスをグリップ
            self.get_logger().info('=== ステップ3: 降下してグリップ ===')
            grip_a = {
                'x': self.point_a['x'],
                'y': self.point_a['y'],
                'z': self.point_a['z'] + self.grip_offset,
                'pitch': math.pi
            }
            if not self.move_to_pose(grip_a, "グリップ位置"):
                return False
            
            # グリップをシミュレート（ボックスをアタッチ）
            self.attach_box("target_box")
            rclpy.spin_once(self, timeout_sec=1.0)
            
            # 5. 持ち上げる
            self.get_logger().info('=== ステップ4: 持ち上げ ===')
            if not self.move_to_pose(approach_a, "持ち上げ"):
                return False
            rclpy.spin_once(self, timeout_sec=1.0)
            
            # 6. B地点上方へ移動
            self.get_logger().info('=== ステップ5: B地点上方へ移動 ===')
            approach_b = {
                'x': self.point_b['x'],
                'y': self.point_b['y'],
                'z': self.point_b['z'] + self.approach_offset,
                'pitch': math.pi
            }
            if not self.move_to_pose(approach_b, "B地点上方"):
                return False
            rclpy.spin_once(self, timeout_sec=1.0)
            
            # 7. 降下して配置
            self.get_logger().info('=== ステップ6: 降下して配置 ===')
            place_b = {
                'x': self.point_b['x'],
                'y': self.point_b['y'],
                'z': self.point_b['z'] + self.grip_offset,
                'pitch': math.pi
            }
            if not self.move_to_pose(place_b, "配置位置"):
                return False
            
            # グリップ解放をシミュレート（ボックスをデタッチ）
            self.detach_box("target_box")
            # ボックスをB地点に再配置
            self.remove_object_from_scene("target_box")
            self.add_box_to_scene("target_box", self.point_b)
            rclpy.spin_once(self, timeout_sec=1.0)
            
            # 8. 上昇
            self.get_logger().info('=== ステップ7: 上昇 ===')
            if not self.move_to_pose(approach_b, "上昇"):
                return False
            rclpy.spin_once(self, timeout_sec=1.0)
            
            # 9. ホームポジションに戻る
            self.get_logger().info('=== ステップ8: ホームポジションに戻る ===')
            if not self.move_to_home():
                return False
            
            self.get_logger().info('=== ピック＆プレース完了！ ===')
            return True
            
        except Exception as e:
            self.get_logger().error(f'エラー発生: {str(e)}')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    try:
        demo = PickAndPlaceDemo()
        
        # ピック＆プレース実行
        success = demo.execute_pick_and_place()
        
        if success:
            demo.get_logger().info('デモ成功！')
        else:
            demo.get_logger().error('デモ失敗')
        
        # ノードを少し維持してRVizで確認
        demo.get_logger().info('10秒間待機（RVizで確認してください）...')
        rclpy.spin_once(demo, timeout_sec=10.0)
        
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
