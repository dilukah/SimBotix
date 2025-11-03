# robot_utils.py
import pybullet as p
import pybullet_data
import math

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

def inverse_kinematics_position_only(robot_id, ee_index, target_pos):
    """Position-only IK."""
    return p.calculateInverseKinematics(robot_id, ee_index, target_pos)

def inverse_kinematics_with_orientation(robot_id, ee_index, target_pos, roll=3.14, pitch=0.0, yaw=0.0):
    """IK with orientation (6D constraint)."""
    quat = p.getQuaternionFromEuler([roll, pitch, yaw])
    return p.calculateInverseKinematics(robot_id, ee_index, target_pos, quat)

# Manual control utilities

def create_manual_sliders():
    """Create sliders for manual control."""
    sx = p.addUserDebugParameter("Target X", -0.8, 0.8, 0.3)
    sy = p.addUserDebugParameter("Target Y", -0.8, 0.8, 0.0)
    sz = p.addUserDebugParameter("Target Z", 0.0, 1.0, 0.6)
    mode_toggle = p.addUserDebugParameter("IK Mode (0=Pos, 1=Ori)", 0, 1, 0)
    sroll = p.addUserDebugParameter("Roll (rad)", -3.14, 3.14, 3.14)
    spitch = p.addUserDebugParameter("Pitch (rad)", -3.14, 3.14, 0.0)
    syaw = p.addUserDebugParameter("Yaw (rad)", -3.14, 3.14, 0.0)
    return sx, sy, sz, mode_toggle, sroll, spitch, syaw

def read_manual_sliders(sx, sy, sz, mode_toggle, sroll, spitch, syaw):
    """Read all slider values."""
    return (
        p.readUserDebugParameter(sx),
        p.readUserDebugParameter(sy),
        p.readUserDebugParameter(sz),
        int(p.readUserDebugParameter(mode_toggle)),
        p.readUserDebugParameter(sroll),
        p.readUserDebugParameter(spitch),
        p.readUserDebugParameter(syaw),
    )

def draw_target_marker(x, y, z):
    """Draw a crosshair target marker."""
    p.addUserDebugLine([x-0.02, y, z], [x+0.02, y, z], [1, 0, 1], 3, 0.05)
    p.addUserDebugLine([x, y-0.02, z], [x, y+0.02, z], [1, 0, 1], 3, 0.05)
    p.addUserDebugLine([x, y, z-0.02], [x, y, z+0.02], [1, 0, 1], 3, 0.05)

def control_gripper(robot_id, open=True, force=20):
    """Open or close the gripper fingers."""
    # Panda finger joints
    gripper_joints = [9, 10]
    target_pos = 0.04 if open else 0.0  # 4cm open, 0cm closed

    for j in gripper_joints:
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=j,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_pos,
            force=force,
        )

def create_gripper_slider():
    """Add manual gripper control slider."""
    return p.addUserDebugParameter("Gripper (0=Close, 1=Open)", 0, 1, 1)

def read_gripper_slider(slider_id):
    """Return True if gripper should be open."""
    val = p.readUserDebugParameter(slider_id)
    return val > 0.5