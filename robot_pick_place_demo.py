import numpy as np
import pybullet as p
import time, random
import robot_utils as ru
from config import CONFIGS

# Setup
config = CONFIGS["panda"]
env = config.get("environment", None)

if env:
    env_no_obj = dict(env)
    if "object" in env_no_obj:
        env_no_obj["object"] = dict(env["object"])
        env_no_obj["object"]["enabled"] = False

ru.connect_gui(env_no_obj)
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)

robot_id, table_height = ru.load_robot(config)
ee_index = config["end_effector_index"]
joint_indices = config["joint_indices"]
workspace = config["workspace_limits"]

for _ in range(240):
    p.stepSimulation()
    time.sleep(1/240)

# Task setup
PLACE_TARGET = [0.55, 0.0, table_height + 0.05]
ru.draw_target_marker(*PLACE_TARGET)

# Move function now records every step
def move_to(target, steps=120, grip_open=1.0, phase=""):
    """Smoothly move EE to target, logging every step."""
    global current_pos
    start = current_pos
    step_data = []

    for i in range(steps):
        alpha = (i + 1) / steps
        pos = [start[j] + alpha * (target[j] - start[j]) for j in range(3)]

        joint_targets = ru.inverse_kinematics_with_orientation(
            robot_id, ee_index, pos, roll=3.14, pitch=0.0, yaw=0.0
        )

        # Apply control
        for j in joint_indices:
            p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL,
                                    joint_targets[j], force=200)
        ru.control_gripper(robot_id, grip_open)
        p.stepSimulation()
        time.sleep(1/240)

        # Record full step
        joint_positions = [p.getJointState(robot_id, j)[0] for j in joint_indices]
        joint_velocities = [p.getJointState(robot_id, j)[1] for j in joint_indices]

        if obj_id is not None:
            obj_pos, _ = p.getBasePositionAndOrientation(obj_id)
            obj_vel, _ = p.getBaseVelocity(obj_id)
        else:
            obj_pos = [0.0, 0.0, 0.0]
            obj_vel = [0.0, 0.0, 0.0]

        step_data.append({
            "phase": phase,
            "joint_positions": joint_positions,
            "joint_velocities": joint_velocities,
            "joint_targets": joint_targets,
            "gripper_open": grip_open,
            "object_pos": obj_pos,
            "object_vel": obj_vel,
            "place_target": PLACE_TARGET,
            "timestamp": time.time(),
            "failed": False
        })

    current_pos = target
    return step_data


# Main demo loop
trial = 0
HOME_JOINTS = [0.0, -0.4, 0.0, -2.5, 0.0, 2.0, 0.8]
home_pose = [0.4, 0.0, table_height + 0.4]

try:
    while True:
        trial += 1
        print(f"Trial {trial} starting...")

        obj_pos = ru.sample_object_position(workspace, table_height)
        obj_id = ru.reset_object(env, obj_pos)
        ru.draw_target_marker(*obj_pos)

        ru.reset_robot_pose(robot_id, joint_indices, HOME_JOINTS)
        ee_state = p.getLinkState(robot_id, ee_index)
        current_pos = list(ee_state[0])
        move_to(home_pose, steps=200, grip_open=1.0)
        ee_state = p.getLinkState(robot_id, ee_index)
        current_pos = list(ee_state[0])

        hover_above = [obj_pos[0], obj_pos[1], obj_pos[2] + 0.25]
        grasp_pose = [obj_pos[0], obj_pos[1], obj_pos[2] + 0.01]
        lift_pose = [obj_pos[0], obj_pos[1], obj_pos[2] + 0.25]
        pre_place = [PLACE_TARGET[0], PLACE_TARGET[1], PLACE_TARGET[2] + 0.2]
        place_pose = [PLACE_TARGET[0], PLACE_TARGET[1], PLACE_TARGET[2] + 0.02]
        retreat_pose = [PLACE_TARGET[0], PLACE_TARGET[1], PLACE_TARGET[2] + 0.25]

        sequence = [
            (hover_above, 1.0, "approach"),
            (grasp_pose, 1.0, "descend"),
            (grasp_pose, 0.0, "grasp"),
            (lift_pose, 0.0, "lift"),
            (pre_place, 0.0, "move_to_target"),
            (place_pose, 0.0, "descend_target"),
            (place_pose, 1.0, "release"),
            (retreat_pose, 1.0, "retreat"),
        ]

        trial_data = []
        for goal, grip, phase in sequence:
            step_data = move_to(goal, steps=150, grip_open=grip, phase=phase)
            trial_data.extend(step_data)

        # Check success
        success = ru.check_success(obj_id, PLACE_TARGET)
        if not success:
            print(f"Trial {trial} failed.")
            for f in trial_data:
                f["failed"] = True
        else:
            print(f"Trial {trial} succeeded.")

        np.savez(f"expert_demo_trial_{trial}.npz", demo_data=trial_data)
        print(f"Saved {len(trial_data)} frames.")
        trial_data.clear()

        for _ in range(240):
            p.stepSimulation()
            time.sleep(1/240)

except KeyboardInterrupt:
    print("Recording stopped.")
