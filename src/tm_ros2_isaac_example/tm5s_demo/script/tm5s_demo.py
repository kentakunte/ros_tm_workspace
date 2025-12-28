from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.stage import add_reference_to_stage

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pathlib import Path
import sys

class Listener(Node):
    def __init__(self, action):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.callback_pos,
            10
        )
        self.subscription
        self.action = action

    def callback_pos(self, data): # ros subscriber callback
        self.action.joint_positions = data.position


if __name__ == "__main__":

    script_dir = Path(__file__).resolve().parent
    root = script_dir.parents[1]
    usd_file = root / "assets" / "worlds" / "tm5s_demo_world.usd"
    print(usd_file)
    if not usd_file.exists():
        print("usd file not found!")
        sys.exit(1)

    my_world = World(stage_units_in_meters=1.0)
    # my_world.scene.add_default_ground_plane()

    # import the robot to stage
    add_reference_to_stage(
        prim_path="/World",
        usd_path=str(usd_file)
        )

    robot_prim_path = "/World/tm5s"
    robot = Robot(name="tm5s", prim_path=robot_prim_path) # get tm5s robot

    my_world.scene.add(robot) # add robot to scene

    articulation_controller = robot.get_articulation_controller() # create an articulation controller for the robot
    
    action = ArticulationAction() # action that is used to get data from /joint_states

    rclpy.init()
    listener = Listener(action)
    
    my_world.reset() # reset world scene

    while simulation_app.is_running() and rclpy.ok():
        rclpy.spin_once(listener, timeout_sec=0.01)
        my_world.step(render=True)

        if my_world.is_playing():

            if my_world.current_time_step_index == 0:
                my_world.reset()
            
            articulation_controller.apply_action(action) # move the robot

    listener.destroy_node()
    rclpy.shutdown()
    simulation_app.close()
