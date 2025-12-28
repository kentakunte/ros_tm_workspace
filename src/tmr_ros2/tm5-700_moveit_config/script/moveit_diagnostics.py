#!/usr/bin/env python3
"""
MoveIt診断スクリプト - インタラクティブマーカーの問題をデバッグ
"""

import rclpy
from rclpy.node import Node
import sys


class MoveItDiagnostics(Node):
    def __init__(self):
        super().__init__('moveit_diagnostics')
        self.create_timer(2.0, self.check_topics_and_tfs)
        
    def check_topics_and_tfs(self):
        self.get_logger().info('=== MoveIt診断情報 ===')
        
        # トピック一覧を取得
        topics = self.get_topic_names_and_types()
        
        # 重要なトピックをチェック
        required_topics = [
            '/joint_states',
            '/move_group/display_planned_path',
            '/planning_scene',
            '/move_group/monitored_planning_scene',
            '/tf',
            '/tf_static',
        ]
        
        self.get_logger().info('--- 必須トピックのチェック ---')
        for topic in required_topics:
            exists = any(t[0] == topic for t in topics)
            status = '✓' if exists else '✗'
            self.get_logger().info(f'{status} {topic}')
        
        # Interactive Marker関連のトピック
        self.get_logger().info('\n--- Interactive Markerトピック ---')
        im_topics = [t[0] for t in topics if 'rviz' in t[0] or 'interactive' in t[0] or 'marker' in t[0]]
        if im_topics:
            for topic in im_topics:
                self.get_logger().info(f'  - {topic}')
        else:
            self.get_logger().warn('Interactive Marker関連のトピックが見つかりません')
        
        # joint_statesをサブスクライブしてチェック
        self.check_joint_states()
        
    def check_joint_states(self):
        """joint_statesトピックの状態をチェック"""
        from sensor_msgs.msg import JointState
        
        def callback(msg):
            self.get_logger().info(f'\n--- Joint States受信 ---')
            self.get_logger().info(f'関節数: {len(msg.name)}')
            self.get_logger().info(f'関節名: {msg.name}')
            if msg.position:
                self.get_logger().info(f'現在位置: {[f"{p:.3f}" for p in msg.position]}')
            else:
                self.get_logger().warn('位置情報が空です！')
        
        self.create_subscription(JointState, '/joint_states', callback, 10)


def main(args=None):
    rclpy.init(args=args)
    node = MoveItDiagnostics()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
