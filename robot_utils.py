# robot_utils.py
import pybullet as p
import pybullet_data
import math

def connect_gui():
    """Connect to PyBullet GUI and load the ground plane."""
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")

def load_robot(config):
    """Load the robot and print joint info."""
    robot_id = p.loadURDF(config["urdf_path"], useFixedBase=config.get("use_fixed_base", True))
    num_joints = p.getNumJoints(robot_id)
    print("=== Robot Joint Info ===")
    for j in range(num_joints):
        info = p.getJointInfo(robot_id, j)
        print(j, info[1].decode("utf-8"))
    return robot_id

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