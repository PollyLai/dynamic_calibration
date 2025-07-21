from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.dynamic_control import _dynamic_control
import numpy as np, csv, os, time

usd_path = "/Robots/UniversalRobots/ur10e/ur10e.usd"
prim_path = "/World/ur10e"
traj_npz = "./trajectory_optmzn/traj_for_isaac/ptrnSrch_N7T25QR.npz"
csv_dir  = "./dataset_ur10e/my_ident_data"
os.makedirs(csv_dir, exist_ok=True)
csv_path = os.path.join(csv_dir, "ptrnSrch_N7T25QR.csv")

world = World()
robot = add_reference_to_stage(usd_path, prim_path)
world.reset()


robot = None
for i in range(120):
    world.step(render=True)
    time.sleep(0.05)
    robot = world.scene.get_object("ur10e")
    if robot is not None and hasattr(robot, 'num_dof') and robot.num_dof == 6:
        print(f"找到機械手臂：{robot.name}, path={robot.prim_path}, DOF={robot.num_dof}")
        break


from pxr import Usd, UsdGeom
stage = world.stage
for prim in stage.Traverse():
    print(prim.GetPath(), prim.GetTypeName())
    
if robot is None:
    raise RuntimeError("❌ 沒找到 articulation robot，請檢查 usd 位置和 prim_path")


