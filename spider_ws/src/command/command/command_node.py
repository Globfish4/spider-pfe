import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
import numpy as np

# from real_spider_articulation import RealSpiderArticulation
from command.real_spider_articulation import RealSpiderArticulation







class CommandPublisher(Node):
    def __init__(self):
        super().__init__("command_publisher")

        self.robot = RealSpiderArticulation()
        self.robot.setup_motors()
        self.goal_pos = {
            "revA_11":0.0, 
            "revB_11":0.0,
            "rev_12":np.pi/6,
            "revA_21":0.0, 
            "revB_21":0.0,
            "rev_22":np.pi/6,
            "revA_31":0.0, 
            "revB_31":0.0, 
            "rev_32":np.pi/6, 
            "revA_41":0.0, 
            "revB_41":0.0, 
            "rev_42":np.pi/6
        }

        self.cmd_publisher_ = self.create_publisher(Float64MultiArray, "position_command", 10)
        self.pos_publisher_ = self.create_publisher(Float64MultiArray, "real_position", 10)
        self.load_publisher_ = self.create_publisher(Float64MultiArray, "present_load", 10)
        self.cmd_subscriber_ = self.create_subscription(Float64MultiArray, "cmd", self.cmd_callback, 10)
        self.on_subscriber_ = self.create_subscription(Bool, '/on', self.on_callback, 10)
        self.timer_ = self.create_timer(0.005, self.timer_callback)
        
        self.cmd_msg = Float64MultiArray()
        self.pos_msg = Float64MultiArray()
        self.load_msg = Float64MultiArray()
        self.counter = 0
        self.get_logger().info("CommandPublisher node started")

    
    def timer_callback(self):

        shut_msgs = self.robot.get_shutdown()
        if len(shut_msgs) > 0:
            self.get_logger().info("overload error : " + str(shut_msgs)) 

        self.cmd_msg.data = list(self.goal_pos.values())
        self.cmd_publisher_.publish(self.cmd_msg)

        self.pos_msg.data = self.robot.get_motors_pos()
        self.pos_publisher_.publish(self.pos_msg)

        self.load_msg.data = self.robot.get_motors_load()
        self.load_publisher_.publish(self.load_msg)

        self.robot.go_to(self.goal_pos)
        # self.get_logger().info(f'Published: {msg.data}')

        
        self.counter += 1


    def cmd_callback(self, msg):
        motors_names = ["revA_11", "revB_11","rev_12","revA_21", "revB_21","rev_22","revA_31", "revB_31", "rev_32", "revA_41", "revB_41", "rev_42"]
        for k in range(len(motors_names)):
            self.goal_pos[motors_names[k]] = msg.data[k]


    def on_callback(self, msg):
        if not msg.data:
            self.get_logger().info("Shutting down CommandPublisher")
            self.robot.shutdown()
            # self.timer_.cancel()
            self.destroy_node()
            rclpy.shutdown()

    





def main(args=None):
    rclpy.init(args=args)
    node = CommandPublisher()
    rclpy.spin(node)
    node.robot.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()





