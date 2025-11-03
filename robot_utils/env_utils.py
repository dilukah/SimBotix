import pybullet as p
import pybullet_data
import numpy as np
import random
import time

def connect_gui(env_config=None):
    """Connect to PyBullet GUI and optionally load environment elements."""
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    if env_config is None:
        # Default: just plane
        p.loadURDF("plane.urdf")
        return

    # Plane
    if env_config.get("plane", True):
        p.loadURDF("plane.urdf")

    # Table
    table_cfg = env_config.get("table", {})
    if table_cfg.get("enabled", False):
        p.loadURDF(
            table_cfg["urdf"],
            table_cfg["position"],
            useFixedBase=True,
        )

    # Object (cube, etc.)
    obj_cfg = env_config.get("object", {})
    if obj_cfg.get("enabled", False):
        p.loadURDF(
            obj_cfg["urdf"],
            obj_cfg["position"],
        )

def load_robot(config):
    """Load robot and environment safely."""
    env = config.get("environment", {})

    # Load plane
    if env.get("plane", True):
        p.loadURDF("plane.urdf")

    # Load table if enabled
    table_z = 0.0
    table_conf = env.get("table", {})
    if table_conf.get("enabled", False):
        p.loadURDF(
            table_conf["urdf"],
            table_conf.get("position", [0.5, 0.0, 0.0])
        )
        table_z = table_conf["position"][2] + 0.63  # ≈ table top height

    # Compute robot base position (above table or on ground)
    base_z = table_z
    base_pos = [0.0, 0.0, base_z]

    # Load robot at correct height
    robot_id = p.loadURDF(
        config["urdf_path"],
        basePosition=base_pos,
        useFixedBase=config.get("use_fixed_base", True)
    )

    # Print joint info
    num_joints = p.getNumJoints(robot_id)
    print("=== Robot Joint Info ===")
    for j in range(num_joints):
        info = p.getJointInfo(robot_id, j)
        print(j, info[1].decode("utf-8"))

    return robot_id, table_z

def sample_place_target(config, table_z):
    limits = config["workspace_limits"]
    x = np.random.uniform(*limits["x"])
    y = np.random.uniform(*limits["y"])
    z = table_z + 0.02
    return [x, y, z]

def draw_place_marker(pos, marker_id=None):
    # marker_id optional: if provided, replace existing lines
    x,y,z = pos
    size = 0.03
    ids = []
    ids.append(p.addUserDebugLine([x-size,y,z],[x+size,y,z],[0,1,0],2,replaceItemUniqueId=marker_id) )
    ids.append(p.addUserDebugLine([x,y-size,z],[x,y+size,z],[0,1,0],2,replaceItemUniqueId=marker_id+1 if marker_id else None))
    return ids  # store to remove/replace later

def reset_object(env_config, obj_pos):
    obj_cfg = env_config.get("object", {})
    if not obj_cfg.get("enabled", False):
        return None

    if "obj_id" not in reset_object.__dict__:
        reset_object.obj_id = p.loadURDF(
            obj_cfg["urdf"],
            obj_pos,
            useFixedBase=False
        )
    else:
        p.resetBasePositionAndOrientation(reset_object.obj_id, obj_pos, [0, 0, 0, 1])

    p.changeDynamics(reset_object.obj_id, -1, mass=0.05, lateralFriction=0.8)
    return reset_object.obj_id


def reset_robot_pose(robot_id, joint_indices, home_joints):
    for j, angle in zip(joint_indices, home_joints):
        p.resetJointState(robot_id, j, angle)
        p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL,
                                targetPosition=angle, force=300)
    for _ in range(240):
        p.stepSimulation()
        time.sleep(1 / 240)


def check_success(obj_id, place_target, threshold=0.05):
    pos, _ = p.getBasePositionAndOrientation(obj_id)
    dist = np.linalg.norm(np.array(pos) - np.array(place_target))
    return dist < threshold

def sample_object_position(workspace, table_height):
    """Sample a random valid object position within the workspace."""
    x = random.uniform(*workspace["x"])
    y = random.uniform(*workspace["y"])
    z = table_height + 0.02
    return [x, y, z]