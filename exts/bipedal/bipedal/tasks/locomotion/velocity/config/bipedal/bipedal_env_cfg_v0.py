# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg
from omni.isaac.lab.envs import ManagerBasedRLEnvCfg
from omni.isaac.lab.managers import CurriculumTermCfg as CurrTerm
from omni.isaac.lab.managers import EventTermCfg as EventTerm
from omni.isaac.lab.managers import ObservationGroupCfg as ObsGroup
from omni.isaac.lab.managers import ObservationTermCfg as ObsTerm
from omni.isaac.lab.managers import RewardTermCfg as RewTerm
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.managers import TerminationTermCfg as DoneTerm
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.terrains import TerrainImporterCfg
from omni.isaac.lab.sensors import ContactSensorCfg, RayCasterCfg, patterns
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import bipedal.tasks.locomotion.velocity.config.bipedal.mdp as mdp

from omni.isaac.lab.managers import CurriculumTermCfg as CurrTerm

##
# Pre-defined configs
##
# from omni.isaac.lab_assets.cartpole import CARTPOLE_CFG  # isort:skip
from bipedal.assets.bipedal import BIPEDAL_CFG  # isort:skip
from omni.isaac.lab.terrains.config.rough import ROUGH_TERRAINS_CFG  # isort: skip
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR

##
# Scene definition
##

@configclass
class BipedalSceneCfg(InteractiveSceneCfg):
    """Configuration for the terrain scene with a legged robot."""

    # ground terrain
    # terrain = TerrainImporterCfg(
    #     prim_path="/World/ground",
    #     terrain_type="generator",
    #     terrain_generator=ROUGH_TERRAINS_CFG,
    #     max_init_terrain_level=5,
    #     collision_group=-1,
    #     physics_material=sim_utils.RigidBodyMaterialCfg(
    #         friction_combine_mode="multiply",
    #         restitution_combine_mode="multiply",
    #         static_friction=1.0,
    #         dynamic_friction=1.0,
    #     ),
    #     visual_material=sim_utils.MdlFileCfg(
    #         mdl_path=f"{ISAACLAB_NUCLEUS_DIR}/Materials/TilesMarbleSpiderWhiteBrickBondHoned/TilesMarbleSpiderWhiteBrickBondHoned.mdl",
    #         project_uvw=True,
    #         texture_scale=(0.25, 0.25),
    #     ),
    #     debug_vis=False,
    # )

    """Configuration for a cart-pole scene."""
    
    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    # cartpole
    robot: ArticulationCfg = BIPEDAL_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")
    # sensors
    contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/robot/.*", history_length=3, track_air_time=True)
    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )
    distant_light = AssetBaseCfg(
        prim_path="/World/DistantLight",
        spawn=sim_utils.DistantLightCfg(color=(0.9, 0.9, 0.9), intensity=2500.0),
        init_state=AssetBaseCfg.InitialStateCfg(rot=(0.738, 0.477, 0.477, 0.0)),
    )


##
# MDP settings
##
@configclass
class CommandsCfg:
    """Command specifications for the MDP."""

    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(10.0, 10.0),
        rel_standing_envs=0.02,
        rel_heading_envs=1.0,
        heading_command=True,
        heading_control_stiffness=0.5,
        debug_vis=True,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(-1.0, 1.0), lin_vel_y=(0.0, 0.0), ang_vel_z=(-1.0, 1.0), heading=(-math.pi, math.pi)
        ),
    )


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    # joint_effort = mdp.JointEffortActionCfg(asset_name="robot", joint_names=["slider_to_cart"], scale=100.0)
    wheel_r_joint_effort = mdp.JointEffortActionCfg(asset_name="robot", joint_names=["wheel_r"], scale=1.0)
    wheel_l_joint_effort = mdp.JointEffortActionCfg(asset_name="robot", joint_names=["wheel_l"], scale=1.0)
    thigh_r_joint_effort = mdp.JointEffortActionCfg(asset_name="robot", joint_names=["thigh_r"], scale=1.0)
    thigh_l_joint_effort = mdp.JointEffortActionCfg(asset_name="robot", joint_names=["thigh_l"], scale=1.0)
    # wheel_r_joint_effort = mdp.JointVelocityActionCfg(asset_name="robot", joint_names=["wheel_r"], scale=1.0)
    # wheel_l_joint_effort = mdp.JointVelocityActionCfg(asset_name="robot", joint_names=["wheel_l"], scale=1.0)
    # thigh_r_joint_effort = mdp.JointPositionActionCfg(asset_name="robot", joint_names=["thigh_r"], scale=1.0)
    # thigh_l_joint_effort = mdp.JointPositionActionCfg(asset_name="robot", joint_names=["thigh_l"], scale=1.0)


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        # joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel)
        # joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel)
        # root_quat_w = ObsTerm(func=mdp.root_quat_w)
        # root_ang_vel_w = ObsTerm(func=mdp.root_ang_vel_w)
        # observation terms (order preserved)
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, noise=Unoise(n_min=-0.1, n_max=0.1))
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.2, n_max=0.2))
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity,
            noise=Unoise(n_min=-0.05, n_max=0.05),
        )
        velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01),)
                            # params={"asset_cfg": SceneEntityCfg("robot", joint_names=["thigh_r", "thigh_l"]),})
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-1.5, n_max=1.5))
        actions = ObsTerm(func=mdp.last_action)
        # height_scan = ObsTerm(
        #     func=mdp.height_scan,
        #     params={"sensor_cfg": SceneEntityCfg("height_scanner")},
        #     noise=Unoise(n_min=-0.1, n_max=0.1),
        #     clip=(-1.0, 1.0),
        # )

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""
    # startup
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.8, 1.0),
            "dynamic_friction_range": (0.6, 0.8),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 64,
        },
    )

    add_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base_link"),
            "mass_distribution_params": (0.0, 3.0),
            "operation": "add",
        },
    )

    # reset
    base_external_force_torque = EventTerm(
        func=mdp.apply_external_force_torque,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base_link"),
            "force_range": (0.0, 0.0),
            "torque_range": (-0.0, 0.0),
        },
    )

    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (-0.5, 0.5),
                "roll": (-0.5, 0.5),
                "pitch": (-0.5, 0.5),
                "yaw": (-0.5, 0.5),
            },
        },
    )

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (0.5, 1.5),
            "velocity_range": (0.0, 0.0),
        },
    )

    # interval
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(10.0, 15.0),
        params={"velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}},
    )



@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # (1) Constant running reward
    # alive = RewTerm(func=mdp.is_alive, weight=1.0)
    # (2) Failure penalty
    terminating = RewTerm(func=mdp.is_terminated, weight=-200.0)
    # (3) Primary task: keep pole upright
    # -- task
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_exp, weight=3.0, params={"command_name": "base_velocity", "std": math.sqrt(0.25)} # 1.0
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_exp, weight=1.5, params={"command_name": "base_velocity", "std": math.sqrt(0.25)} # 0.5
    )
    # -- penalties
    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=-2.0)  
    ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05)
    dof_torques_l2 = RewTerm(func=mdp.joint_torques_l2, weight=-1.0e-5)
    # dof_torques_l2_thigh = RewTerm(func=mdp.joint_torques_l2, weight=-1.0e-3, 
    #                          params={"asset_cfg": SceneEntityCfg("robot", joint_names=["thigh_r", "thigh_l"]),})
    # dof_torques_l2_wheel = RewTerm(func=mdp.joint_torques_l2, weight=-1.0e-5, 
    #                          params={"asset_cfg": SceneEntityCfg("robot", joint_names=["wheel_r", "wheel_l"]),})
    dof_acc_l2 = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-0.01)
    # feet_air_time = RewTer2m(
    #     func=mdp.feet_air_time,
    #     weight=0.125,
    #     params={
    #         "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*FOOT"),
    #         "command_name": "base_velocity",
    #         "threshold": 0.5,
    #     },
    # )
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-5.0,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=["calf_r", "calf_l"]), "threshold": 1.0},
    )
    # -- optional penalties
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-5.0)    # -2.0
    dof_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-1.0)    # -2.0
    # joint_pos_dif_l1 = RewTerm(func=mdp.joint_pos_dif_l1, weight=-0.001)
    joint_target_deviation_range_l1 = RewTerm(
        func=mdp.joint_target_deviation_range_l1,
        weight=5.0,
        params={
            "min_angle": -1.321,
            "max_angle": -1.221,
            "in_range_reward": 0.0,
            "asset_cfg": SceneEntityCfg("robot", joint_names=["thigh_r", "thigh_l"]),
        },  # target: -1.271
    )



@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    # (1) Time out
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    # (2) Cart out of bounds
    # thigh_r_out_of_bounds = DoneTerm(
    #     func=mdp.joint_pos_out_of_manual_limit,
    #     params={"asset_cfg": SceneEntityCfg("robot", joint_names=["thigh_r"]), "bounds": (-0.785, -1.57)},
    # )
    # thigh_l_out_of_bounds = DoneTerm(
    #     func=mdp.joint_pos_out_of_manual_limit,
    #     params={"asset_cfg": SceneEntityCfg("robot", joint_names=["thigh_l"]), "bounds": (-0.785, -1.57)},
    # )
    # torso_angular = DoneTerm(func=mdp.bad_orientation, params={"limit_angle": 0.53})
    base_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=["calf_r", "calf_l", "base_link"]), "threshold": 1.0},
    )


@configclass
class CurriculumCfg:
    """Configuration for the curriculum."""
    # terrain_levels = CurrTerm(func=mdp.terrain_levels_vel)
    pass


##
# Environment configuration
##


@configclass
class BipedalEnvCfgV0(ManagerBasedRLEnvCfg):
    """Configuration for the locomotion velocity-tracking environment."""

    # Scene settings
    scene: BipedalSceneCfg = BipedalSceneCfg(num_envs=4096, env_spacing=2.5)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    # MDP settings
    curriculum: CurriculumCfg = CurriculumCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    # No command generator
    commands: CommandsCfg = CommandsCfg()

    # Post initialization
    def __post_init__(self) -> None:
        """Post initialization."""
        # # general settings
        # self.decimation = 2
        # self.episode_length_s = 20.
        # # viewer settings
        # self.viewer.eye = (8.0, 0.0, 5.0)
        # # simulation settings
        # self.sim.dt = 1 / 120
        # self.sim.render_interval = self.decimation
        # general settings
        self.decimation = 4     #4
        self.episode_length_s = 20.0
        # simulation settings
        self.sim.dt = 0.005     # 0.005
        self.sim.render_interval = self.decimation
        self.sim.disable_contact_processing = True
        # self.sim.physics_material = self.scene.terrain.physics_material
        # # update sensor update periods
        # # we tick all the sensors based on the smallest update period (physics update period)
        # if self.scene.height_scanner is not None:
        #     self.scene.height_scanner.update_period = self.decimation * self.sim.dt
        if self.scene.contact_forces is not None:
            self.scene.contact_forces.update_period = self.sim.dt
