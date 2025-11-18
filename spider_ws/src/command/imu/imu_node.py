import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
import numpy as np
import math

# from real_spider_articulation import RealSpiderArticulation
from command.real_spider_articulation import RealSpiderArticulation

from pymavlink import mavutil






# --- Paramètres connexion ---
PORT = '/dev/ttyACM0'  # Linux : /dev/ttyACM0, Windows : COM3
BAUD = 115200

# --- Constantes capteur (MPU6000) ---
ACCEL_SCALE = 1024.0   # LSB/g pour ±16g
GYRO_SCALE = 16.4      # LSB/(°/s) pour ±2000°/s
G_TO_MS2 = 9.80665     # Conversion g -> m/s²








class IMUReader(Node):
    def __init__(self):
        super().__init__("imu_reader")

        # --- Connexion MAVLink ---
        print(f"Connexion à {PORT}...")
        self.master = mavutil.mavlink_connection(PORT, baud=BAUD)

        print("Attente du heartbeat...")
        self.master.wait_heartbeat()
        print(f"Connecté à système {self.master.target_system}, composant {self.master.target_component}")

        # --- Demande de flux pour RAW_IMU et ATTITUDE ---
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,  # inclut RAW_IMU
            20,  # fréquence en Hz
            1    # start
        )

        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,  # inclut ATTITUDE
            20,  # fréquence en Hz
            1    # start
        )



        self.raw_imu_publisher_ = self.create_publisher(Float64MultiArray, "imu", 10)
        self.attitude_publisher_ = self.create_publisher(Float64MultiArray, "attitude", 10)
        self.timer_ = self.create_timer(0.05, self.timer_callback)
        
        self.raw_imu_msg = Float64MultiArray()
        self.attitude_msg = Float64MultiArray()
        self.counter = 0
        self.get_logger().info("CommandPublisher node started")

    
    def timer_callback(self):

        msg = self.master.recv_match(blocking=True)
        if msg:
            if msg.get_type() == "RAW_IMU":
                # Conversion en unités physiques
                ax = msg.xacc / ACCEL_SCALE * G_TO_MS2
                ay = msg.yacc / ACCEL_SCALE * G_TO_MS2
                az = msg.zacc / ACCEL_SCALE * G_TO_MS2

                gx = (msg.xgyro / GYRO_SCALE) * (math.pi / 180.0)
                gy = (msg.ygyro / GYRO_SCALE) * (math.pi / 180.0)
                gz = (msg.zgyro / GYRO_SCALE) * (math.pi / 180.0)

                self.raw_imu_msg.data = [ax, ay, az, gx, gy, gz]


            elif msg.get_type() == "ATTITUDE":
                roll_deg = math.degrees(msg.roll)
                pitch_deg = math.degrees(msg.pitch)
                yaw_deg = math.degrees(msg.yaw)

                roll_speed = msg.rollspeed
                pitch_speed = msg.pitchspeed
                yaw_speed = msg.yawspeed

                self.attitude_msg.data = [roll_deg, pitch_deg, yaw_deg, roll_speed, pitch_speed, yaw_speed]
        
        self.counter += 1

        self.raw_imu_publisher_.publish(self.raw_imu_msg)
        self.attitude_publisher_.publish(self.attitude_msg)







def main(args=None):
    rclpy.init(args=args)
    node = IMUReader()

    rclpy.spin(node)
    node.robot.shutdown()
    # node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


