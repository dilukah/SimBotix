import time
import pybullet as p
from robot_configs import ROBOT_CONFIGS
from robot_utils import (
    connect_gui,
    load_robot,
    inverse_kinematics_position_only,
    inverse_kinematics_with_orientation,
    create_manual_sliders,
    read_manual_sliders,
    draw_target_marker,
)

# Setup
config = ROBOT_CONFIGS["panda"]
connect_gui()
robot_id = load_robot(config)
ee_index = config["end_effector_index"]
joint_indices = config["joint_indices"]

# Create sliders
sx, sy, sz, mode_toggle, sroll, spitch, syaw = create_manual_sliders()

# Main simulation loop
while True:
    p.stepSimulation()
    x, y, z, ik_mode, roll, pitch, yaw = read_manual_sliders(sx, sy, sz, mode_toggle, sroll, spitch, syaw)

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

    draw_target_marker(x, y, z)
    time.sleep(1.0 / 240.0)