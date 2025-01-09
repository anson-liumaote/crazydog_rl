# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Cartpole balancing environment.
"""

import gymnasium as gym

from . import agents
from .bipedal_env_cfg_v0 import BipedalEnvCfgV0
from .bipedal_env_cfg_v1 import BipedalEnvCfgV1
from .bipedal_env_cfg_v2 import BipedalEnvCfgV2
from .bipedal_env_cfg_v3 import BipedalEnvCfgV3
from .bipedal_env_cfg_v4 import BipedalEnvCfgV4
from .bipedal_env_cfg_v5 import BipedalEnvCfgV5
from .bipedal_env_cfg_v6 import BipedalEnvCfgV6
##
# Register Gym environments.
##

gym.register(
    id="Isaac-Bipedal-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": BipedalEnvCfgV0,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:BipedalPPORunnerCfg",
    },
)
gym.register(
    id="Isaac-Bipedal-v1",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": BipedalEnvCfgV1,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:BipedalPPORunnerCfg",
    },
)
gym.register(
    id="Isaac-Bipedal-v2",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": BipedalEnvCfgV2,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:BipedalPPORunnerCfg",
    },
)
gym.register(
    id="Isaac-Bipedal-v3",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": BipedalEnvCfgV3,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:BipedalPPORunnerCfg",
    },
)
gym.register(
    id="Isaac-Bipedal-v4",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": BipedalEnvCfgV4,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:BipedalPPORunnerCfg",
    },
)
gym.register(
    id="Isaac-Bipedal-v5",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": BipedalEnvCfgV5,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:BipedalPPORunnerCfg",
    },
)
gym.register(
    id="Isaac-Bipedal-v6",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": BipedalEnvCfgV6,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:BipedalPPORunnerCfg",
    },
)