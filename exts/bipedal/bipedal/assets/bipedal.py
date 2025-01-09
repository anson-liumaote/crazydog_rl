# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for a simple Cartpole robot."""


import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg, DelayedPDActuatorCfg, DCMotorCfg, ActuatorNetMLPCfg
from omni.isaac.lab.assets import ArticulationCfg
from omni.isaac.lab.utils.assets import ISAACLAB_NUCLEUS_DIR
from bipedal.assets import ISAACLAB_ASSETS_DATA_DIR

##
# Configuration
##

BIPEDAL_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/csl/Downloads/bipedal_new/urdf/bipedal_new/bipedal_new.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
    ),
    
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.1), 
        joint_pos={"wheel_r": 0.0, "wheel_l": 0.0, "thigh_r": -1.27, "thigh_l": -1.27,},
        # joint_vel={"wheel_r": 0.0, "wheel_l": 0.0, "thigh_r": 0.0, "thigh_l": 0.0,},

    ),
    # soft_joint_pos_limit_factor=0.9,
    actuators={
        "wheel_r_actuator": ImplicitActuatorCfg(
            joint_names_expr=["wheel_r"],
            effort_limit=2.46/2,
            velocity_limit=61.47/2,
            # min_delay=0,  # physics time steps (min: 5.0 * 0 = 0.0ms)
            # max_delay=4,
            stiffness=0.0,
            damping=0.0, # damping=0.3,
        ),
        "wheel_l_actuator": ImplicitActuatorCfg(
            joint_names_expr=["wheel_l"],
            effort_limit=2.46/2,
            velocity_limit=61.47/2,
            # min_delay=0,  # physics time steps (min: 5.0 * 0 = 0.0ms)
            # max_delay=4,
            stiffness=0.0,
            damping=0.0, # damping=0.3,
        ),
        
        "thigh_r_actuator": ImplicitActuatorCfg(
            joint_names_expr=["thigh_r"],
            effort_limit=23.5/2,
            velocity_limit=30.0/2,
            # min_delay=0,  # physics time steps (min: 5.0 * 0 = 0.0ms)
            # max_delay=4,
            stiffness=0.0, # stiffness=25.0,
            damping=0.0, # damping=0.5,
        ),
        "thigh_l_actuator": ImplicitActuatorCfg(
            joint_names_expr=["thigh_l"],
            effort_limit=23.5/2,
            velocity_limit=30.0/2,
            # min_delay=0,  # physics time steps (min: 5.0 * 0 = 0.0ms)
            # max_delay=4,
            stiffness=0.0, # stiffness=25.0,
            damping=0.0, # damping=0.5,
        ),
    },
)


BIPEDAL_CFG_V1 = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/csl/Downloads/crazydog_urdf/urdf/crazydog_urdf/crazydog_urdf.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
    ),
    
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.5),
        joint_pos={
            ".*_hip2thigh": 1.271, 
            ".*_thigh2calf": -2.12773, 
            ".*_calf2wheel": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    # soft_joint_pos_limit_factor=0.9,
    actuators={
        "unitree_actuator": DelayedPDActuatorCfg(
            joint_names_expr=[".*_hip2thigh", ".*_thigh2calf"],
            effort_limit=23.5/2,
            velocity_limit=30.0/2,
            min_delay=0,  # physics time steps (min: 5.0 * 0 = 0.0ms)
            max_delay=4,
            stiffness=0.0,
            damping=0.0, # damping=0.3,
        ),
        "wheel_actuator": DelayedPDActuatorCfg(
            joint_names_expr=[".*_calf2wheel"],
            effort_limit=2.46/2,
            velocity_limit=61.47/2,
            min_delay=0,  # physics time steps (min: 5.0 * 0 = 0.0ms)
            max_delay=4,
            stiffness=0.0, # stiffness=25.0,
            damping=0.0, # damping=0.5,
        ),
    },
)

BIPEDAL_CFG_V2 = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/csl/Downloads/crazydog_urdf/urdf/crazydog_urdf/crazydog_urdf.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
    ),
    
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.5),
        joint_pos={
            ".*_hip2thigh": 1.271, 
            ".*_thigh2calf": -2.12773, 
            ".*_calf2wheel": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "thigh_actuator": DelayedPDActuatorCfg(
            joint_names_expr=[".*_hip2thigh", ".*_thigh2calf"],
            effort_limit=23.5,
            velocity_limit=30.0,
            min_delay=0,  # physics time steps (min: 5.0 * 0 = 0.0ms)
            max_delay=4,
            stiffness=25.0,
            damping=0.5, # damping=0.3,
        ),        
        "wheel_actuator": DelayedPDActuatorCfg(
            joint_names_expr=[".*_calf2wheel"],
            effort_limit=2.46/2,
            velocity_limit=61.47/2,
            min_delay=0,  # physics time steps (min: 5.0 * 0 = 0.0ms)
            max_delay=4,
            stiffness=0.0, # stiffness=25.0,
            damping=0.3, # damping=0.5,
        ),
    },
)

GO1_ACTUATOR_CFG = ActuatorNetMLPCfg(
    joint_names_expr=[".*_hip2thigh"],
    network_file=f"{ISAACLAB_NUCLEUS_DIR}/ActuatorNets/Unitree/unitree_go1.pt",
    pos_scale=-1.0, # defult: -1.0
    vel_scale=1.0,
    torque_scale=1.0,
    input_order="pos_vel",
    input_idx=[0, 1, 2],
    effort_limit=23.7,  # taken from spec sheet
    velocity_limit=30.0,  # taken from spec sheet
    saturation_effort=23.7,  # same as effort limit
)
BIPEDAL_CFG_V3 = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/csl/Downloads/crazydog_urdf/urdf/crazydog_urdf/crazydog_urdf.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
    ),
    
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.5),
        joint_pos={
            ".*_hip2thigh": 1.271, 
            ".*_thigh2calf": -2.12773, 
            ".*_calf2wheel": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "unitree_actuator": GO1_ACTUATOR_CFG,      
        "wheel_actuator": DelayedPDActuatorCfg(
            joint_names_expr=[".*_calf2wheel"],
            effort_limit=2.46/2,
            velocity_limit=61.47/2,
            min_delay=0,  # physics time steps (min: 5.0 * 0 = 0.0ms)
            max_delay=4,
            stiffness=0.0, # stiffness=25.0,
            damping=0.3, # damping=0.5,
        ),
    },
)

# only wheel actuators
BIPEDAL_CFG_V4 = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/csl/Downloads/crazydog_urdf/urdf/crazydog_urdf_fix/crazydog_urdf_fix.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
    ),
    
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.3),
        joint_pos={
            ".*_calf2wheel": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "wheel_actuator": DelayedPDActuatorCfg(
            joint_names_expr=[".*_calf2wheel"],
            effort_limit=2.46/2,
            velocity_limit=61.47/2,
            min_delay=0,  # physics time steps (min: 5.0 * 0 = 0.0ms)
            max_delay=4,
            stiffness=0.0, # stiffness=0.0,
            damping=0.05, # damping=0.3,
        ),
    },
)

# only wheel actuators
BIPEDAL_CFG_V5 = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAACLAB_ASSETS_DATA_DIR}/Robots/Bipedal/bi_urdf_2dof/bi_urdf_2dof.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
    ),
    
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.4),
        joint_pos={
            ".*_calf2wheel": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "wheel_actuator": DelayedPDActuatorCfg(
            joint_names_expr=[".*_calf2wheel"],
            effort_limit=2.46/2,
            velocity_limit=61.47/2,
            min_delay=0,  # physics time steps (min: 5.0 * 0 = 0.0ms)
            max_delay=4,
            stiffness=0.0, # stiffness=0.0,
            damping=0.1, # damping=0.3,
        ),
    },
)


# 4 dof bipedal robot
BIPEDAL_CFG_V6 = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAACLAB_ASSETS_DATA_DIR}/Robots/Bipedal/bi_urdf_4dof/bi_urdf_4dof.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
    ),
    
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.4),
        joint_pos={
            ".*_hip2thigh": 1.2235425, 
            ".*_calf2wheel": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "unitree_actuator": GO1_ACTUATOR_CFG, 
        "wheel_actuator": DelayedPDActuatorCfg(
            joint_names_expr=[".*_calf2wheel"],
            effort_limit=2.46/2,
            velocity_limit=61.47/2,
            min_delay=0,  # physics time steps (min: 5.0 * 0 = 0.0ms)
            max_delay=4,
            stiffness=0.0,  # stiffness=0.0,
            damping=0.1,  # damping=0.3,
        ),
    },
)