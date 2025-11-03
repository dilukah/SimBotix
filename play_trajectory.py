import numpy as np
import pybullet as p
import pybullet_data
import time
from config import CONFIGS
import robot_utils as ru

#skip frames for faster playback
SKIP_FRAMES = 5

# Load demo
demo_file = "expert_demo_trial_1.npz"  # change to your file
data = np.load(demo_file, allow_pickle=True)
frames = data["demo_data"].tolist()

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

# Draw place target
PLACE_TARGET = frames[0]["place_target"]
ru.draw_target_marker(*PLACE_TARGET)

# Draw initial object
initial_obj_pos = frames[0]["object_pos"]
obj_id = ru.reset_object(env, initial_obj_pos)


# Animate frames
for i, frame in enumerate(frames):
    if i % SKIP_FRAMES != 0:
        continue
    joint_positions = frame["joint_positions"]
    gripper_open = frame["gripper_open"]
    failed = frame.get("failed", False)

    # Smoothly apply joints over several simulation steps
    for _ in range(4):  # small interpolation for smoothness
        for j, pos in zip(joint_indices, joint_positions):
            p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL, targetPosition=pos, force=200)
        ru.control_gripper(robot_id, gripper_open)
        p.stepSimulation()
        time.sleep(1/240)

    # Draw marker above place target with color depending on success/fail
    color = [1, 0, 0] if failed else [0, 1, 0]
    p.addUserDebugLine([PLACE_TARGET[0]-0.03, PLACE_TARGET[1], PLACE_TARGET[2]+0.05],
                       [PLACE_TARGET[0]+0.03, PLACE_TARGET[1], PLACE_TARGET[2]+0.05],
                       color, 2, 0.1)

# Keep GUI open
while True:
    p.stepSimulation()
    #time.sleep(1/240)
