import time
from math import sin, cos
import pybullet as p
from robot_configs import ROBOT_CONFIGS
from robot_utils import (
    connect_gui,
    load_robot,
    inverse_kinematics_position_only,
    inverse_kinematics_with_orientation,
)

# Setup
config = ROBOT_CONFIGS["panda"]
connect_gui()
robot_id = load_robot(config)
ee_index = config["end_effector_index"]
joint_indices = config["joint_indices"]

# Create sliders for manual control
sx = p.addUserDebugParameter("Target X", -0.8, 0.8, 0.3)
sy = p.addUserDebugParameter("Target Y", -0.8, 0.8, 0.0)
sz = p.addUserDebugParameter("Target Z", 0.0, 1.0, 0.6)

# Add toggle: 0 = position-only, 1 = position+orientation
mode_toggle = p.addUserDebugParameter("IK Mode (0=Pos, 1=Ori)", 0, 1, 0)

# Optional sliders for orientation (only used if orientation mode is active)
sroll = p.addUserDebugParameter("Roll (rad)", -3.14, 3.14, 3.14)
spitch = p.addUserDebugParameter("Pitch (rad)", -3.14, 3.14, 0.0)
syaw = p.addUserDebugParameter("Yaw (rad)", -3.14, 3.14, 0.0)

# Run Simulation
while True:
    p.stepSimulation()

    # sliders (manual control)
    x = p.readUserDebugParameter(sx)
    y = p.readUserDebugParameter(sy)
    z = p.readUserDebugParameter(sz)

    # Read toggle and orientation sliders
    ik_mode = int(p.readUserDebugParameter(mode_toggle))
    roll = p.readUserDebugParameter(sroll)
    pitch = p.readUserDebugParameter(spitch)
    yaw = p.readUserDebugParameter(syaw)

    # Compute IK
    if ik_mode == 0:
        joint_poses = inverse_kinematics_position_only(robot_id, ee_index, [x, y, z])
    else:
        joint_poses = inverse_kinematics_with_orientation(robot_id, ee_index, [x, y, z], roll, pitch, yaw)

    # Apply joint positions
    for j in joint_indices:
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=j,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_poses[j],
            force=200,
        )

    # Draw crosshair target (unchanged)
    p.addUserDebugLine([x-0.02, y, z], [x+0.02, y, z], [1, 0, 1], 3, 0.05)
    p.addUserDebugLine([x, y-0.02, z], [x, y+0.02, z], [1, 0, 1], 3, 0.05)
    p.addUserDebugLine([x, y, z-0.02], [x, y, z+0.02], [1, 0, 1], 3, 0.05)

    time.sleep(1.0 / 240.0)