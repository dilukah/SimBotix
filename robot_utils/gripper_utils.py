import pybullet as p

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