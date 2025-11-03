import math

CONFIGS = {
    "panda": {
        # --- Robot settings ---
        "urdf_path": "franka_panda/panda.urdf",
        "end_effector_index": 11,
        "joint_indices": list(range(7)),
        "use_fixed_base": True,

        # --- Environment elements ---
        "environment": {
            "plane": True,

            "table": {
                "enabled": True,  # toggle for table
                "urdf": "table/table.urdf",
                "position": [0.5, 0.0, 0.0],  # move table slightly in front
            },

            "object": {
                "enabled": True,  # toggle for pick-and-place object
                "urdf": "cube_small.urdf",
                "position": [0.6, 0.0, 0.63 + 0.02],
            },
        },

        # --- Workspace bounds (for IK limits, sampling, RL goals) ---
        "workspace_limits": {
            "x": (0.2, 0.8),
            "y": (-0.4, 0.4),
            "z": (0.3, 0.8),
        },

        # --- Default orientation (for orientation IK) ---
        "default_orientation": (math.pi, 0, 0),
    },

    "kuka": {
        # --- Robot settings ---
        "urdf_path": "kuka_iiwa/model.urdf",
        "end_effector_index": 6,
        "joint_indices": list(range(7)),
        "use_fixed_base": True,
        "base_offset": [0.0, 0.0, 0.0],
        "default_orientation": (0, math.pi, 0),

        # --- Environment elements ---
        "environment": {
            "plane": True,

            "table": {
                "enabled": False,  # start without table
                "urdf": "table/table.urdf",
                "position": [0.6, 0.0, 0.0],
            },

            "object": {
                "enabled": False,
                "urdf": "cube_small.urdf",
                "position": [0.7, 0.0, 0.6],
            },
        },

        # --- Workspace bounds ---
        "workspace_limits": {
            "x": (0.3, 0.9),
            "y": (-0.4, 0.4),
            "z": (0.0, 0.8),
        },
    },
}
