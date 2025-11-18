# Copyright (c) 2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

##
# Configuration for different assets.
##


import gymnasium as gym

from . import agents
from .create_spider_direct_rl_cfg import SpiderEnvCfg

##
# Register Gym environments.
##


gym.register(
    id="Spider-Direct-v0",
    entry_point=f"{__name__}.create_spider_direct_rl_cfg:SpiderEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.create_spider_direct_rl_cfg:SpiderEnvCfg",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:spiderPPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
    },
)

gym.register(
    id="Spider-Turn-Direct-v0",
    entry_point=f"{__name__}.create_spider_direct_rl_cfg_turn:SpiderEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.create_spider_direct_rl_cfg_turn:SpiderEnvCfg",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:spiderPPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
    },
)
