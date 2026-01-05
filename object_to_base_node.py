import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
import traceback

class ObjectToBaseNode(Node):
    def __init__(self):
        super().__init__('object_to_base_node')
        print("ObjectToBaseNode initializing...")
        self.get_logger().info("Starting ObjectToBaseNode initialization")
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Wait for TF buffer to fill
        self.get_logger().info("Waiting for TF buffer to fill (3 seconds)...")
        import time
        time.sleep(3.0)
        
        # Initialize MoveIt Action Client
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self.motion_in_progress = False
        self.last_move_time = None
        self.move_cooldown = 3.0  # Wait 3 seconds between moves
        
        self.get_logger().info("Waiting for MoveIt move_group action server...")
        if self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("✓ Connected to MoveIt move_group action server")
            self.moveit_available = True
        else:
            self.get_logger().warn("✗ MoveIt move_group action server not available")
            self.get_logger().warn("Run: ros2 launch tm5-700_moveit_config demo.launch.py use_sim_time:=true")
            self.moveit_available = False
        
        # Reachability checker with motion planning
        self.last_check_time = None
        self.check_interval = 1.0  # Check every 1 second
        
        self.get_logger().info("Reachability Checker Mode - Will report if box is reachable")
        self.get_logger().info("Workspace limits: 0.15-1.5m distance, -0.5 to 1.5m height")
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_red = np.array([0, 150, 50])
            upper_red = np.array([10, 255, 255])
            mask = cv2.inRange(hsv, lower_red, upper_red)
            M = cv2.moments(mask)

            if M["m00"] > 0:
                # Throttle checking to avoid spam
                now = self.get_clock().now()
                if self.last_check_time is not None:
                    time_since_check = (now - self.last_check_time).nanoseconds / 1e9
                    if time_since_check < self.check_interval:
                        return
                
                self.last_check_time = now
                
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                
                f, Z = 900, 0.6
                x_cam = (cX - 640) * Z / f
                y_cam = (cY - 360) * Z / f
                z_cam = Z

                point_cam = PointStamped()
                point_cam.header.frame_id = 'eih_2d_camera'
                point_cam.header.stamp = self.get_clock().now().to_msg()
                point_cam.point.x, point_cam.point.y, point_cam.point.z = x_cam, y_cam, z_cam

                # Try to transform - check multiple possible base frame names
                base_frames = ['base', 'world', 'base_link', 'tm_base_link']
                point_base = None
                
                for base_frame in base_frames:
                    try:
                        # Use time=0 to get the latest available transform
                        transform = self.tf_buffer.lookup_transform(
                            base_frame,
                            'eih_2d_camera',
                            rclpy.time.Time(),
                            rclpy.duration.Duration(seconds=0.5)
                        )
                        point_cam.header.stamp = rclpy.time.Time().to_msg()
                        point_base = self.tf_buffer.transform(point_cam, base_frame)
                        
                        # Success - use this frame
                        if not hasattr(self, 'base_frame_found'):
                            self.get_logger().info(f"✓ Found base frame: '{base_frame}'")
                            self.base_frame_found = base_frame
                        break
                    except Exception:
                        continue
                
                if point_base is None:
                    # No valid frame found
                    if not hasattr(self, 'tf_error_logged'):
                        self.get_logger().error(f"Could not find TF transform. Tried frames: {base_frames}")
                        self.get_logger().error("Is Isaac Sim running? Are TF transforms being published?")
                        self.tf_error_logged = True
                    return

                # Check if object has moved significantly
                current_position = (point_base.point.x, point_base.point.y, point_base.point.z)
                
                # Check if position is within reachable workspace
                distance_from_base = (current_position[0]**2 + current_position[1]**2)**0.5
                height = current_position[2]
                
                # Define workspace limits
                min_distance = 0.15
                max_distance = 1.5
                min_height = -0.5
                max_height = 1.5
                
                # Check reachability
                distance_ok = min_distance <= distance_from_base <= max_distance
                height_ok = min_height <= height <= max_height
                is_reachable = distance_ok and height_ok
                
                # Report results
                self.get_logger().info("="*60)
                self.get_logger().info("RED BOX DETECTED")
                self.get_logger().info(f"Position in world frame: x={current_position[0]:.3f}m, y={current_position[1]:.3f}m, z={current_position[2]:.3f}m")
                self.get_logger().info(f"Distance from origin: {distance_from_base:.3f}m")
                self.get_logger().info(f"Height: {height:.3f}m")
                self.get_logger().info("-"*60)
                
                if is_reachable:
                    self.get_logger().info("✅ BOX IS REACHABLE")
                    self.get_logger().info(f"   - Distance OK: {distance_from_base:.3f}m (range: {min_distance}-{max_distance}m)")
                    self.get_logger().info(f"   - Height OK: {height:.3f}m (range: {min_height}-{max_height}m)")
                else:
                    self.get_logger().warn("❌ BOX IS NOT REACHABLE")
                    if not distance_ok:
                        if distance_from_base < min_distance:
                            self.get_logger().warn(f"   - Too close: {distance_from_base:.3f}m < {min_distance}m")
                        else:
                            self.get_logger().warn(f"   - Too far: {distance_from_base:.3f}m > {max_distance}m")
                    if not height_ok:
                        if height < min_height:
                            self.get_logger().warn(f"   - Too low: {height:.3f}m < {min_height}m")
                        else:
                            self.get_logger().warn(f"   - Too high: {height:.3f}m > {max_height}m")
                
                self.get_logger().info("="*60)
                
                # If reachable and MoveIt available, plan and execute motion
                if is_reachable and self.moveit_available and not self.motion_in_progress:
                    # Check cooldown
                    now = self.get_clock().now()
                    if self.last_move_time is not None:
                        time_since_move = (now - self.last_move_time).nanoseconds / 1e9
                        if time_since_move < self.move_cooldown:
                            self.get_logger().debug(f"Cooldown active: {time_since_move:.1f}s / {self.move_cooldown}s")
                            return
                    
                    # Create target pose 20cm above box
                    target_pose = PoseStamped()
                    target_pose.header.frame_id = self.base_frame_found
                    target_pose.pose.position.x = current_position[0]
                    target_pose.pose.position.y = current_position[1]
                    target_pose.pose.position.z = current_position[2] + 0.20  # 20cm above box
                    
                    # Set any valid orientation (will be determined by IK)
                    target_pose.pose.orientation.w = 1.0
                    target_pose.pose.orientation.x = 0.0
                    target_pose.pose.orientation.y = 0.0
                    target_pose.pose.orientation.z = 0.0
                    
                    self.get_logger().info(f"📍 Moving to position 20cm above box...")
                    self.motion_in_progress = True
                    self.last_move_time = now
                    self.send_move_goal(target_pose)
                
            else:
                # No box detected
                pass  # Silent when no box

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")
    
    def send_move_goal(self, target_pose: PoseStamped):
        """Send a motion planning goal to MoveIt"""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "tmr_arm"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1  # 10% of max velocity
        goal_msg.request.max_acceleration_scaling_factor = 0.1  # 10% of max acceleration
        
        # Workspace parameters
        goal_msg.request.workspace_parameters.header.frame_id = target_pose.header.frame_id
        goal_msg.request.workspace_parameters.min_corner.x = -1.5
        goal_msg.request.workspace_parameters.min_corner.y = -1.5
        goal_msg.request.workspace_parameters.min_corner.z = -0.5
        goal_msg.request.workspace_parameters.max_corner.x = 1.5
        goal_msg.request.workspace_parameters.max_corner.y = 1.5
        goal_msg.request.workspace_parameters.max_corner.z = 1.5
        
        # Set pose goal using constraints
        constraints = Constraints()
        
        # Position constraint
        position_constraint = PositionConstraint()
        position_constraint.header = target_pose.header
        position_constraint.link_name = "link_6"
        
        bounding_volume = BoundingVolume()
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.1, 0.1, 0.1]  # 10cm tolerance - more relaxed
        bounding_volume.primitives.append(box)
        bounding_volume.primitive_poses.append(target_pose.pose)
        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0
        
        constraints.position_constraints.append(position_constraint)
        
        # Note: Orientation constraint removed to ensure IK solution exists
        # The robot will reach the position in whatever orientation is achievable
        
        goal_msg.request.goal_constraints.append(constraints)
        
        self.get_logger().info(f"Sending goal: ({target_pose.pose.position.x:.2f}, {target_pose.pose.position.y:.2f}, {target_pose.pose.position.z:.2f})")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle the response from MoveIt"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('❌ Goal rejected by MoveIt!')
                self.motion_in_progress = False
                return

            self.get_logger().info('✓ Goal accepted, planning and executing...')
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)
        except Exception as e:
            self.get_logger().error(f'Error in goal response: {e}')
            self.motion_in_progress = False

    def get_result_callback(self, future):
        """Handle the final result from MoveIt"""
        self.motion_in_progress = False
        try:
            result = future.result().result
            if result.error_code.val == 1:  # SUCCESS
                self.get_logger().info('✅ Motion executed successfully!')
            else:
                self.get_logger().error(f'❌ Motion failed with error code: {result.error_code.val}')
        except Exception as e:
            self.get_logger().error(f'Error getting result: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectToBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()