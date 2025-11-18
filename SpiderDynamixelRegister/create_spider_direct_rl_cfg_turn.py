
import torch
import sys 
import os
from collections.abc import Sequence

import isaacsim.core.utils.torch as torch_utils
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.terrains import TerrainImporterCfg

import gymnasium as gym
# import isaaclab_tasks  # noqa: F401
from isaacsim.core.utils.torch.rotations import compute_heading_and_up, compute_rot, quat_conjugate

sys.path.append(os.environ['HOME'] + "/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/SpiderRegister/")







# Configuration
SPIDER_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        # usd_path=f"_isaac_sim/CAMILO/robots/spiderLimited.usd",
        # usd_path="_isaac_sim/CAMILO/dynamixel/usd/spiderLimited.usd",
        # usd_path="_isaac_sim/CAMILO/dynamixel/usd/spider_real_limited.usd",
        usd_path="_isaac_sim/CAMILO/dynamixel/usd/spider_real_short_limited.usd",
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
        copy_from_source=False,
        collision_props=sim_utils.CollisionPropertiesCfg(
            collision_enabled=True,
        ),
        mass_props=sim_utils.MassPropertiesCfg(mass=1.5)
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0, 0, 0.3), 
        # joint_pos={
        #     "revA_11": 0.0, "revA_21": 0.0, "revA_31": 0.0, "revA_41": 0.0, 
        #     "revB_11": -torch.pi/8, "revB_21": -torch.pi/8, "revB_31": -torch.pi/8, "revB_41": -torch.pi/8, 
        #     "rev_12": 3*torch.pi/8, "rev_22": 3*torch.pi/8, "rev_32": 3*torch.pi/8, "rev_42": 3*torch.pi/8
        #     },
        joint_pos={
            "revA_11": 0.0, "revB_11": -torch.pi/8, "rev_12": 3*torch.pi/8,
            "revA_21": 0.0, "revB_21": -torch.pi/8, "rev_22": 3*torch.pi/8,
            "revA_31": 0.0, "revB_31": -torch.pi/8, "rev_32": 3*torch.pi/8,
            "revA_41": 0.0, "revB_41": -torch.pi/8, "rev_42": 3*torch.pi/8,
        },
        rot=(0.9238795325112867, 0, 0, 0.3826834323650898)   #  rot=(1, 0, 0, 0.0),

    ),
    # actuators={
    #     "Hips": ImplicitActuatorCfg(
    #         joint_names_expr=["revA_.*"],
    #         stiffness=0.0,
    #         damping=0.0,
    #     ),
    #     "Quads": ImplicitActuatorCfg(
    #         joint_names_expr=["revB_.*"],
    #         stiffness=0.0,
    #         damping=0.0,
    #     ),
    #     "Calves": ImplicitActuatorCfg(
    #         joint_names_expr=["rev_.*"],
    #         stiffness=0.0,
    #         damping=0.0,
    #     ),
    # },
    actuators={
        "Hips": ImplicitActuatorCfg(
            joint_names_expr=["revA_.*"],
            stiffness=15,
            damping=0.29,
            effort_limit_sim=15,
            velocity_limit=2.4,
        ),
        "Quads": ImplicitActuatorCfg(
            joint_names_expr=["revB_.*"],
            stiffness=15,
            damping=0.29,
            effort_limit_sim=15,
            velocity_limit=2.4,
        ),
        "Calves": ImplicitActuatorCfg(
            joint_names_expr=["rev_.*"],
            stiffness=15,
            damping=0.29,
            effort_limit_sim=15,
            velocity_limit=2.4,
        ),
    },
)





@configclass
class SpiderEnvCfg(DirectRLEnvCfg):
    # env
    # episode_length_steps = ceil(episode_length_s / (decimation_rate * physics_time_step))
    episode_length_s = 15.0
    decimation = 2
    action_scale = 2.0#1.0 #0.5
    action_space = 12
    observation_space = 48#36
    state_space = 0



    # simulation
    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=decimation)

    # terrain
    terrain: TerrainImporterCfg = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="min",
            restitution_combine_mode="average",
            static_friction=1.0,
            dynamic_friction=1.0,
            
            restitution=0.0,
        ),
        debug_vis=False,
    )

    # scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4096, env_spacing=1.0, replicate_physics=True)

    # robot
    robot: ArticulationCfg = SPIDER_CFG.replace(prim_path="/World/envs/env_.*/Robot") # type: ignore

    joint_gears: list = [15, 15, 15, 15, 15, 15, 15, 15, 15, 15., 15, 15]
    joint_gears: list = [70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70]
    # joint_gears: list = [50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50]
    # joint_gears: list = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]

    heading_weight: float = 0.5
    up_weight: float = 0.1

    energy_cost_scale: float = 0.005 #/ 3.0
    # dof_limit_scale: float = 1 / 20.0
    actions_cost_scale: float = 0.005
    alive_reward_scale: float = 0.5
    dof_vel_scale: float = 0.2

    death_cost: float = -2.0
    termination_height: float = 0.01
    termination_height_up: float =  1.5

    angular_velocity_scale: float = 1.0
    contact_force_scale: float = 0.1










def normalize_angle(x):
    return torch.atan2(torch.sin(x), torch.cos(x))


class SpiderEnv(DirectRLEnv):
    cfg: SpiderEnvCfg

    def __init__(self, cfg: SpiderEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        # Motor control
        self.action_space = self.cfg.action_space
        self.action_scale = self.cfg.action_scale
        self.joint_gears = torch.tensor(self.cfg.joint_gears, dtype=torch.float32, device=self.sim.device)
        self.motor_effort_ratio = torch.ones_like(self.joint_gears, device=self.sim.device)
        self._joint_dof_idx, _ = self.robot.find_joints(".*")

        # Targets
        self.potentials = torch.zeros(self.num_envs, dtype=torch.float32, device=self.sim.device)
        self.prev_potentials = torch.zeros_like(self.potentials)
        self.targets = torch.tensor([1000, 0, 0], dtype=torch.float32, device=self.sim.device).repeat(self.num_envs, 1)
        self.targets += self.scene.env_origins # Target that the spider must reach

        # Body properties
        self.start_rotation = torch.tensor([1, 0, 0, 0], device=self.sim.device, dtype=torch.float32)
        self.up_vec = torch.tensor([0, 0, 1], dtype=torch.float32, device=self.sim.device).repeat((self.num_envs, 1))
        self.heading_vec = torch.tensor([1, 0, 0], dtype=torch.float32, device=self.sim.device).repeat(
            (self.num_envs, 1)
        )
        self.inv_start_rot = quat_conjugate(self.start_rotation).repeat((self.num_envs, 1))
        self.basis_vec0 = self.heading_vec.clone()
        self.basis_vec1 = self.up_vec.clone()



        # The following are useful for position control
        self.robot_dof_targets = self.robot.data.joint_pos
        self.dt = self.cfg.sim.dt * self.cfg.decimation
        self.robot_dof_lower_limits = self.robot.data.soft_joint_pos_limits[0, :, 0].to(device=self.device)
        self.robot_dof_upper_limits = self.robot.data.soft_joint_pos_limits[0, :, 1].to(device=self.device)

        self.robot_dof_speed_scales = torch.ones_like(self.robot_dof_lower_limits)

        self.angle_tracker = AngleTrackerTorch(quat_to_yaw_deg(self.robot.data.root_quat_w))
        self.tracked_angle = self.angle_tracker.update(quat_to_yaw_deg(self.robot.data.root_quat_w))




    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot)

        self.cfg.terrain.num_envs = self.scene.cfg.num_envs
        self.cfg.terrain.env_spacing = self.scene.cfg.env_spacing
        self.terrain = self.cfg.terrain.class_type(self.cfg.terrain)
        
        # clone and replicate
        self.scene.clone_environments(copy_from_source=False)
        # add articulation and objects to scene
        self.scene.articulations["robot"] = self.robot
        # add lights
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)



    def _pre_physics_step(self, actions: torch.Tensor) -> None:

        self.actions = actions.clone().clamp(-1.0, 1.0)
        targets = self.robot_dof_targets + self.robot_dof_speed_scales * self.dt * self.actions * self.cfg.action_scale * self.joint_gears
        self.robot_dof_targets[:] = torch.clamp(targets, self.robot_dof_lower_limits, self.robot_dof_upper_limits)


    def _apply_action(self):    
        self.robot.set_joint_position_target(self.robot_dof_targets)



    # def _pre_physics_step(self, actions: torch.Tensor) -> None:
        # self.actions = actions.clone() # self.action_scale * actions.clone()


    # def _apply_action(self):
        # forces = self.action_scale * self.joint_gears * self.actions
        # self.robot.set_joint_effort_target(forces, joint_ids=self._joint_dof_idx)
        


    def _compute_intermediate_values(self):

        self.tracked_angle = self.angle_tracker.update(quat_to_yaw_deg(self.robot.data.root_quat_w))
        # print(self.tracked_angle)

        self.torso_position = self.robot.data.root_pos_w
        self.torso_rotation = self.robot.data.root_quat_w
        self.velocity = self.robot.data.root_lin_vel_w
        self.ang_velocity = self.robot.data.root_ang_vel_w
        self.dof_pos = self.robot.data.joint_pos
        self.dof_vel = self.robot.data.joint_vel

        (
            self.up_proj, 
            self.heading_proj, 
            self.up_vec, 
            self.heading_vec, 
            self.vel_loc, 
            self.angvel_loc, 
            self.roll, 
            self.pitch, 
            self.yaw, 
            self.angle_to_target, 
            self.dof_pos_scaled, 
            self.prev_potentials, 
            self.potentials, 
        ) = compute_intermediate_values(
            self.targets, 
            self.torso_position, 
            self.torso_rotation, 
            self.velocity, 
            self.ang_velocity, 
            self.dof_pos, 
            self.robot.data.soft_joint_pos_limits[0, :, 0],
            self.robot.data.soft_joint_pos_limits[0, :, 1],
            self.inv_start_rot, 
            self.basis_vec0, 
            self.basis_vec1, 
            self.potentials, 
            self.prev_potentials, 
            self.cfg.sim.dt,
        ) 




    def _get_observations(self) -> dict:
        obs = torch.cat(
            (
                self.torso_position[:, 2].view(-1, 1),
                self.vel_loc,
                self.angvel_loc * self.cfg.angular_velocity_scale,
                normalize_angle(self.yaw).unsqueeze(-1),
                normalize_angle(self.roll).unsqueeze(-1),
                normalize_angle(self.angle_to_target).unsqueeze(-1),
                self.up_proj.unsqueeze(-1),
                self.heading_proj.unsqueeze(-1),
                self.dof_pos_scaled,
                self.dof_vel * self.cfg.dof_vel_scale,
                self.actions,
            ),
            dim=-1,
        )
        observations = {"policy": obs}

        return observations


    def _get_rewards(self) -> torch.Tensor:
        total_reward, dico= compute_rewards(
            self.actions,
            self.reset_terminated,
            self.cfg.up_weight, 
            self.cfg.heading_weight, 
            self.heading_proj, 
            self.up_proj, 
            self.dof_vel, 
            self.dof_pos_scaled, 
            self.potentials, 
            self.prev_potentials, 
            self.cfg.actions_cost_scale, 
            self.cfg.energy_cost_scale, 
            self.cfg.dof_vel_scale, 
            self.cfg.death_cost, 
            self.cfg.alive_reward_scale, 
            self.motor_effort_ratio, 
            self.tracked_angle,
            # self.cfg.dof_limit_scale
            self.angle_tracker.get_delta()
        )

        self.extras["log"] = dico
        return total_reward


    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]: 
        self._compute_intermediate_values()
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        died2 = torch.isnan(self.actions[:, 0])
        died1 = self.torso_position[:, 2] < self.cfg.termination_height
        died3 = self.torso_position[:, 2] > self.cfg.termination_height_up
        died = died2 + died3 + died1
        return died, time_out
    



    def _reset_idx(self, env_ids: torch.Tensor | None):
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self.robot._ALL_INDICES
        self.robot.reset(env_ids)
        super()._reset_idx(env_ids)

        joint_pos = self.robot.data.default_joint_pos[env_ids]
        joint_vel = self.robot.data.default_joint_vel[env_ids]
        default_root_state = self.robot.data.default_root_state[env_ids]
        default_root_state[:, :3] += self.scene.env_origins[env_ids]

        self.robot.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
        self.robot.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)

        to_target = self.targets[env_ids] - default_root_state[:, :3]
        to_target[:, 2] = 0.0
        self.potentials[env_ids] = -torch.norm(to_target, p=2, dim=-1) / self.cfg.sim.dt


        self._compute_intermediate_values()




@torch.jit.script
def compute_rewards(
    actions: torch.Tensor,
    reset_terminated: torch.Tensor,
    up_weight: float,
    heading_weight: float,
    heading_proj: torch.Tensor,
    up_proj: torch.Tensor,
    dof_vel: torch.Tensor,
    dof_pos_scaled: torch.Tensor,
    potentials: torch.Tensor,
    prev_potentials: torch.Tensor,
    actions_cost_scale: float,
    energy_cost_scale: float,
    dof_vel_scale: float,
    death_cost: float,
    alive_reward_scale: float,
    motor_effort_ratio: torch.Tensor,
    tracked_angle: torch.Tensor,
    delta: torch.Tensor,
    # dof_limit_scale: float,
):
    # heading_weight_tensor = torch.ones_like(heading_proj) * heading_weight
    # heading_reward = torch.where(heading_proj > 0.8, heading_weight_tensor, heading_weight * heading_proj / 0.8)

    # print("total angle : ", tracked_angle)
    turning_reward = delta * 0.05
    # print("turning_reward : ", turning_reward)
    # print("turning reward : ", turning_reward)


    # aligning up axis of robot and environment
    up_reward = torch.zeros_like(heading_proj)
    up_reward = torch.where(up_proj > 0.93, up_reward + up_weight, up_reward)

    # energy penalty for movement
    actions_cost = torch.sum(actions**2, dim=-1)
    electricity_cost = torch.sum(
        torch.abs(actions * dof_vel * dof_vel_scale) * motor_effort_ratio.unsqueeze(0),
        dim=-1,
    )

    # dof at limit cost
    dof_at_limit_cost = torch.sum(dof_pos_scaled > 0.98, dim=-1)

    # reward for duration of staying alive
    alive_reward = torch.ones_like(potentials) * alive_reward_scale
    # progress_reward = potentials - prev_potentials


    total_reward = (
        # progress_reward
        # alive_reward
        up_reward
        + turning_reward
        - actions_cost_scale * actions_cost
        # - energy_cost_scale * electricity_cost
        # - dof_at_limit_cost * dof_limit_scale
    )
    # adjust reward for fallen agents
    total_reward = torch.where(reset_terminated, torch.ones_like(total_reward) * death_cost, total_reward) 

    dict_rewards = {
        # "progress_reward": progress_reward, 
        "alive_reward": alive_reward, 
        "up_reward": up_reward, 
        "turning_reward": turning_reward, 
        "actions_cost": actions_cost * actions_cost_scale, 
        "electricity_cost": electricity_cost * energy_cost_scale, 
        "dof_at_limit_cost": dof_at_limit_cost
        }
    # print("dict rewards : ", dict_rewards)

    return total_reward, dict_rewards



@torch.jit.script
def compute_intermediate_values(
    targets: torch.Tensor,
    torso_position: torch.Tensor,
    torso_rotation: torch.Tensor,
    velocity: torch.Tensor,
    ang_velocity: torch.Tensor,
    dof_pos: torch.Tensor,
    dof_lower_limits: torch.Tensor,
    dof_upper_limits: torch.Tensor,
    inv_start_rot: torch.Tensor,
    basis_vec0: torch.Tensor,
    basis_vec1: torch.Tensor,
    potentials: torch.Tensor,
    prev_potentials: torch.Tensor,
    dt: float,
):
    to_target = targets - torso_position
    to_target[:, 2] = 0.0

    torso_quat, up_proj, heading_proj, up_vec, heading_vec = compute_heading_and_up(
        torso_rotation, inv_start_rot, to_target, basis_vec0, basis_vec1, 2
    )

    vel_loc, angvel_loc, roll, pitch, yaw, angle_to_target = compute_rot(
        torso_quat, velocity, ang_velocity, targets, torso_position
    )

    dof_pos_scaled = torch_utils.maths.unscale(dof_pos, dof_lower_limits, dof_upper_limits)

    to_target = targets - torso_position
    to_target[:, 2] = 0.0
    prev_potentials[:] = potentials
    potentials = -torch.norm(to_target, p=2, dim=-1) / dt

    return (
        up_proj,
        heading_proj,
        up_vec,
        heading_vec,
        vel_loc,
        angvel_loc,
        roll,
        pitch,
        yaw,
        angle_to_target,
        dof_pos_scaled,
        prev_potentials,
        potentials,
    )







class AngleTrackerTorch:
    def __init__(self, angle_initial: torch.Tensor):
        """
        angle_initial : tenseur (n, 1) ou (n,) contenant les angles initiaux en degrés
        """
        self.last_angle = angle_initial.clone()
        self.total = torch.zeros_like(angle_initial, dtype=torch.float32)
        self.delta = 0

    def update(self, angle: torch.Tensor) -> torch.Tensor:
        """
        Met à jour avec un tenseur d'angles [0,360) et retourne les cumuls.
        angle : (n, 1) ou (n,) en degrés
        """
        self.delta = angle - self.last_angle

        # correction wrap-around
        self.delta = torch.where(self.delta > 180, self.delta - 360, self.delta)
        self.delta = torch.where(self.delta < -180, self.delta + 360, self.delta)

        self.total += self.delta
        self.last_angle = angle.clone()
        return self.total
    
    def get_delta(self):
        return self.delta






def quat_to_yaw_deg(q: torch.Tensor) -> torch.Tensor:
    """
    Convertit un batch de quaternions (n,4) en angles de rotation autour de Z (yaw) en degrés.
    Convention: quaternion normalisé.
    
    Args:
        q : torch.Tensor de taille (n, 4) avec (w, x, y, z)
    Returns:
        torch.Tensor de taille (n,) contenant les angles en degrés
    """
    w, x, y, z = q.unbind(dim=1)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)

    yaw = torch.atan2(siny_cosp, cosy_cosp)  # radians
    return torch.rad2deg(yaw)