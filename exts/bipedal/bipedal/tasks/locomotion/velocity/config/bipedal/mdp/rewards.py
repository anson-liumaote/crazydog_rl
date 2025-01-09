# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.utils.math import wrap_to_pi

if TYPE_CHECKING:
    from omni.isaac.lab.envs import ManagerBasedRLEnv


def joint_pos_target_l2(env: ManagerBasedRLEnv, target: float, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize joint position deviation from a target value."""
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    # wrap the joint positions to (-pi, pi)
    joint_pos = wrap_to_pi(asset.data.joint_pos[:, asset_cfg.joint_ids])
    # compute the reward
    return torch.sum(torch.square(joint_pos - target), dim=1)

def joint_pos_dif_l1(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Calculate the difference between joint angular position."""
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    # print(asset.data.joint_pos)
    thigh_r_pos = asset.data.joint_pos[:, 0]  
    thigh_l_pos = asset.data.joint_pos[:, 1]  
    # print(thigh_r_pos, thigh_l_pos)
    
    return torch.sum(torch.abs(thigh_r_pos-thigh_l_pos), dim=0)

def joint_target_deviation_range_l1(
    env: ManagerBasedRLEnv,
    min_angle: float,
    max_angle: float,
    in_range_reward: float,
    cmd_threshold: float = -1.0,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Provide a fixed reward when the joint angle is within a specified range and penalize deviations."""
    asset: Articulation = env.scene[asset_cfg.name]
    cmd = torch.norm(env.command_manager.get_command("base_velocity"), dim=1)

    # Get the current joint positions
    current_joint_pos = asset.data.joint_pos[:, asset_cfg.joint_ids]

    # Check if the joint angles are within the specified range
    in_range = (current_joint_pos >= min_angle) & (current_joint_pos <= max_angle)

    # Calculate the absolute deviation from the nearest range limit when out of range
    out_of_range_penalty = torch.abs(
        current_joint_pos - torch.where(current_joint_pos < min_angle, min_angle, max_angle)
    )

    if cmd_threshold != -1.0:
        joint_deviation_range = torch.where(
            cmd.unsqueeze(1) <= cmd_threshold,
            torch.where(in_range, in_range_reward * torch.ones_like(current_joint_pos), -out_of_range_penalty),
            torch.tensor(0.0),
        )
    else:
        # Assign a fixed reward if in range, and a negative penalty if out of range
        joint_deviation_range = torch.where(
            in_range, in_range_reward * torch.ones_like(current_joint_pos), -out_of_range_penalty
        )

    # Sum the rewards over all joint ids
    return torch.sum(joint_deviation_range, dim=1)