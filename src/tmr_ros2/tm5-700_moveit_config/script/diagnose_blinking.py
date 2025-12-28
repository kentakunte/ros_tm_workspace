#!/usr/bin/env python3
"""
RViz点滅問題の診断スクリプト
複数のjoint_state配信者を検出
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from collections import defaultdict
import time


class JointStateMonitor(Node):
    def __init__(self):
        super().__init__('joint_state_monitor')
        
        self.joint_states = defaultdict(list)
        self.last_print = time.time()
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.get_logger().info('=== Joint State Monitor 起動 ===')
        self.get_logger().info('複数の配信者がいる場合、点滅の原因になります')
        
    def joint_state_callback(self, msg):
        # タイムスタンプから発信者を推定
        timestamp = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
        
        # 位置情報を記録
        if msg.position:
            positions_str = ', '.join([f'{p:.3f}' for p in msg.position])
            self.joint_states[positions_str].append(timestamp)
        
        # 1秒ごとに統計を表示
        if time.time() - self.last_print > 1.0:
            self.print_statistics()
            self.last_print = time.time()
            self.joint_states.clear()
    
    def print_statistics(self):
        if not self.joint_states:
            return
            
        self.get_logger().info('\n--- 過去1秒間のJoint State統計 ---')
        
        if len(self.joint_states) == 1:
            self.get_logger().info('✓ 正常: 単一の状態のみ配信されています')
        else:
            self.get_logger().warn(f'⚠ 警告: {len(self.joint_states)}種類の異なる状態が配信されています')
            self.get_logger().warn('これが点滅の原因です！')
        
        for idx, (positions, timestamps) in enumerate(self.joint_states.items(), 1):
            self.get_logger().info(f'\n状態{idx}: [{positions}]')
            self.get_logger().info(f'  配信回数: {len(timestamps)}回')


def main(args=None):
    rclpy.init(args=args)
    
    monitor = JointStateMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
