import pybullet as p
import pybullet_data

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