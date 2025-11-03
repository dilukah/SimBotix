import numpy as np
import pybullet as p
import time
import robot_utils as ru
from config import CONFIGS
import random

# Setup
config = CONFIGS["panda"]
env = config.get("environment", None)

ru.connect_gui(env)
robot_id, table_height = ru.load_robot(config)
ee_index = config["end_effector_index"]
joint_indices = config["joint_indices"]

# Create sliders
sx, sy, sz, mode_toggle, sroll, spitch, syaw = ru.create_manual_sliders()
s_grip = ru.create_gripper_slider()  # new gripper control slider

demo_data = []
table_enabled = env and env.get("table", {}).get("enabled", False)

print("Recording demo... Move sliders to control robot. Press Ctrl+C to stop.")

try:
    trial = 0
    while True:
        p.stepSimulation()

        # Read slider values
        x, y, z, ik_mode, roll, pitch, yaw = ru.read_manual_sliders(
            sx, sy, sz, mode_toggle, sroll, spitch, syaw
        )
        grip_open = ru.read_gripper_slider(s_grip)

        # Adjust for table height if needed
        if table_enabled:
            z += table_height

        # Compute IK
        if ik_mode == 0:
            joint_targets = ru.inverse_kinematics_position_only(robot_id, ee_index, [x, y, z])
        else:
            joint_targets = ru.inverse_kinematics_with_orientation(robot_id, ee_index, [x, y, z], roll, pitch, yaw)

        # Apply joint controls
        for j in joint_indices:
            p.setJointMotorControl2(
                bodyIndex=robot_id,
                jointIndex=j,
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_targets[j],
                force=200,
            )

        # Apply gripper control
        ru.control_gripper(robot_id, grip_open)

        # Visualize target
        ru.draw_target_marker(x, y, z)

        # Record data
        joint_states = [p.getJointState(robot_id, j)[0] for j in joint_indices]
        demo_data.append({
            "joint_positions": joint_states,
            "gripper_open": grip_open,
            "target": [x, y, z, roll, pitch, yaw],
            "ik_mode": ik_mode,
            "timestamp": time.time(),
        })

        time.sleep(1/240)

except KeyboardInterrupt:
    np.savez(f"expert_demo_trial_{trial}.npz", demo_data=demo_data)
    print(f"\nSaved {len(demo_data)} frames to expert_demo_trial_{trial}.npz")