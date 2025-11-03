import pybullet as p

def inverse_kinematics_position_only(robot_id, ee_index, target_pos):
    """Position-only IK."""
    return p.calculateInverseKinematics(robot_id, ee_index, target_pos)

def inverse_kinematics_with_orientation(robot_id, ee_index, target_pos, roll=3.14, pitch=0.0, yaw=0.0):
    """IK with orientation (6D constraint)."""
    quat = p.getQuaternionFromEuler([roll, pitch, yaw])
    return p.calculateInverseKinematics(robot_id, ee_index, target_pos, quat)