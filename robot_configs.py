import math
ROBOT_CONFIGS = {
    "panda": {
        "urdf_path": "franka_panda/panda.urdf",
        "end_effector_index": 11,
        "joint_indices": list(range(7)),
        "use_fixed_base": True,
    },
    "kuka": {
        "urdf_path": "kuka_iiwa/model.urdf",
        "end_effector_index": 6,
        "base_offset": [0.0, 0.0, 0.0],
        "default_orientation": (0, math.pi, 0),
        "workspace_limits": {"x": (0.3, 0.9), "y": (-0.4, 0.4), "z": (0.0, 0.8)},
        "joint_indices": list(range(7)),
        "use_fixed_base": True, 
    },
}