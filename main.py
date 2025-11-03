import time
import pybullet as p
from config import CONFIGS
from robot_utils import (
    connect_gui,
    load_robot,
    inverse_kinematics_position_only,
    inverse_kinematics_with_orientation,
    create_manual_sliders,
    read_manual_sliders,
    draw_target_marker,
    read_gripper_slider,
    create_gripper_slider,
    control_gripper,
)

# Setup
config = CONFIGS["panda"]
connect_gui(config.get("environment", None))
robot_id, table_height = load_robot(config)
ee_index = config["end_effector_index"]
joint_indices = config["joint_indices"]
env = config.get("environment", {})
table_enabled = env.get("table", {}).get("enabled", False)

# Create sliders
sx, sy, sz, mode_toggle, sroll, spitch, syaw = create_manual_sliders()
sgrip = create_gripper_slider() 

# Main simulation loop
while True:
    p.stepSimulation()
    x, y, z, ik_mode, roll, pitch, yaw = read_manual_sliders(sx, sy, sz, mode_toggle, sroll, spitch, syaw)
    if table_enabled:
        z += table_height

    grip_open = read_gripper_slider(sgrip)

    # Compute IK
    if ik_mode == 0:
        joint_poses = inverse_kinematics_position_only(robot_id, ee_index, [x, y, z])
    else:
        joint_poses = inverse_kinematics_with_orientation(robot_id, ee_index, [x, y, z], roll, pitch, yaw)

    # Apply joints
    for j in joint_indices:
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=j,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_poses[j],
            force=200,
        )
    control_gripper(robot_id, open=grip_open)
    draw_target_marker(x, y, z)
    time.sleep(1.0 / 240.0)