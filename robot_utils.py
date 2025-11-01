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
    robot_id = p.loadURDF(config["urdf_path"], useFixedBase=config["use_fixed_base"])
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