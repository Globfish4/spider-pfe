import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
import numpy as np
import torch
from rsl_rl.modules import ActorCritic

from ament_index_python.packages import get_package_share_directory
import os

from nn_controller.rotations import *

# from real_spider_articulation import RealSpiderArticulation
# from command.real_spider_articulation import RealSpiderArticulation





class NNController(Node):
    def __init__(self):
        super().__init__("nn_controller")




        # Paramètres identiques à ceux utilisés pendant l'entraînement
        obs_dim = 48
        act_dim = 12
        actor_hidden_dims = [400, 200, 100]
        critic_hidden_dims = [400, 200, 100]

        # Création du modèle
        self.policy = ActorCritic(
            num_actor_obs=obs_dim,
            num_critic_obs=obs_dim,
            num_actions=act_dim,
            actor_hidden_dims=actor_hidden_dims,
            critic_hidden_dims=critic_hidden_dims,
            activation='elu',
        )

        # Chargement des poids
        pkg_share = get_package_share_directory('command')
        model_path = os.path.join(pkg_share, 'models', 'model_1999.pt')

        checkpoint = torch.load(model_path, map_location=torch.device('cuda:0'))
        self.policy.load_state_dict(checkpoint["model_state_dict"])

        # Mode evaluation
        self.policy.eval()



        self.joint_pos_subscriber_ = self.create_subscription(Float64MultiArray, "real_position", self.joint_pos_callback, 10)
        self.imu_raw_subscriber_ = self.create_subscription(Float64MultiArray, "imu", self.imu_callback, 10)
        self.attitude_subscriber_ = self.create_subscription(Float64MultiArray, "attitude", self.attitude_callback, 10)
        self.cmd_publisher_ = self.create_publisher(Float64MultiArray, "cmd", 10)
        self.timer_ = self.create_timer(0.05, self.timer_callback)
        
        self.att_msg = Float64MultiArray()
        self.pos_msg = Float64MultiArray()
        self.imu_msg = Float64MultiArray()
        self.counter = 0
        self.get_logger().info("nn_controller node started")

    
    def timer_callback(self):
        pass
        # phi, theta, psi = self.att_msg.data[0], self.att_msg.data[1], self.att_msg.data[2], 
        # torso_quat = from_euler_to_quat(phi, theta, psi)
        # velocity = torch.tensor([[0.0, 0.0, 0.0]], dtype=torch.float32, device="cuda:0")
        # ang_velocity = torch.tensor([self.att_msg[3:]])#.view(-1, 1)
        # targets = torch.tensor([[1000, 0, 0]], dtype=torch.float32, device="cuda:0")
        # torso_position = torch.tensor([[0.0, 0.0, 0.0]], dtype=torch.float32, device="cuda:0")

        # vel_loc, angvel_loc, roll, pitch, yaw, angle_to_target = compute_rot(
        # torso_quat, velocity, ang_velocity, targets, torso_position
        # )

        # torso_position = torso_position.view(-1, 1)
        # vel_loc = torch.tensor([[0.0, 0.0, 0.0]]).view(-1, 1)
        # angvel_loc = angvel_loc * self.angular_velocity_scale
        # yaw = normalize_angle(yaw).unsqueeze(-1)
        # roll = normalize_angle(roll).unsqueeze(-1)
        # angle_to_target = normalize_angle(angle_to_target).unsqueeze(-1)
        # up_proj = 
        # heading_proj = 
        # dof_pos_scaled = 
        # dof_vel = 
        # actions = 

        # obs =   (self.torso_position[:, 2].view(-1, 1),
        #         self.vel_loc,
        #         self.angvel_loc * self.cfg.angular_velocity_scale,
        #         normalize_angle(self.yaw).unsqueeze(-1),
        #         normalize_angle(self.roll).unsqueeze(-1),
        #         normalize_angle(self.angle_to_target).unsqueeze(-1),
        #         self.up_proj.unsqueeze(-1),
        #         self.heading_proj.unsqueeze(-1),
        #         self.dof_pos_scaled,
        #         self.dof_vel * self.cfg.dof_vel_scale,
        #         self.actions,)
        

        # action = self.policy.act_inference(obs.unsqueeze(0))
        # message.data = action
        
        message = Float64MultiArray()
        message.data = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.cmd_publisher_.publish(message)

        self.counter += 1


    def attitude_callback(self, msg):
        self.att_msg.data = msg


    def imu_callback(self, msg):
        self.pos_msg.data = msg

    
    def joint_pos_callback(self, msg):
        self.imu_msg.data = msg


    
    def filter(self):
        pass








def main(args=None):
    rclpy.init(args=args)
    node = NNController()
    rclpy.spin(node)
    node.robot.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":





    main()




