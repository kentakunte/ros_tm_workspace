import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

class IsaacActionBridge(Node):
    def __init__(self):
        super().__init__('isaac_action_bridge')
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/tmr_arm_controller/follow_joint_trajectory',
            self.execute_callback
        )
        self.publisher = self.create_publisher(JointState, '/joint_command', 10)
        self.get_logger().info('Action Server Bridge Started. Waiting for MoveIt...')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing trajectory...')
        trajectory = goal_handle.request.trajectory

        # Send the points to Isaac Sim
        for point in trajectory.points:
            msg = JointState()
            msg.header = trajectory.header
            msg.name = trajectory.joint_names
            msg.position = point.positions
            self.publisher_.publish(msg)

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        return result
    
def main(args=None):
    rclpy.init(args=args)
    node = IsaacActionBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
