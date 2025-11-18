from dynamixel_python import DynamixelManager
import time
import numpy as np



# set the model of dynamixel you are using as written in the motor's web url
DYNAMIXEL_MODEL = ['xl430-w250', '2xl430-w250']







class RealSpiderArticulation:
    def __init__(self):
        """
        set up and initialize motors
        """
        self.joints_limits = {
            "revA_11":(1479, 2616),
            "revB_11":(1479, 2616),
            "rev_12":(2389, 3185),
            "revA_21":(1479, 2616),
            "revB_21":(1479, 2616),
            "rev_22":(2389, 3185),
            "revA_31":(1479, 2616),
            "revB_31":(1479, 2616),
            "rev_32":(2389, 3185),
            "revA_41":(1479, 2616),
            "revB_41":(1479, 2616),
            "rev_42":(2389, 3185)
        }

        self.motors = DynamixelManager(baud_rate=115200, usb_port='/dev/ttyUSB0')
        self.motors.add_dynamixel('revA_11', 1, DYNAMIXEL_MODEL[1])
        self.motors.add_dynamixel('revB_11', 2, DYNAMIXEL_MODEL[1])
        self.motors.add_dynamixel('rev_12', 3, DYNAMIXEL_MODEL[0])
        self.motors.add_dynamixel('revA_21', 4, DYNAMIXEL_MODEL[1])
        self.motors.add_dynamixel('revB_21', 5, DYNAMIXEL_MODEL[1])
        self.motors.add_dynamixel('rev_22', 6, DYNAMIXEL_MODEL[0])
        self.motors.add_dynamixel('revA_31', 7, DYNAMIXEL_MODEL[1])
        self.motors.add_dynamixel('revB_31', 8, DYNAMIXEL_MODEL[1])
        self.motors.add_dynamixel('rev_32', 9, DYNAMIXEL_MODEL[0])
        self.motors.add_dynamixel('revA_41', 10, DYNAMIXEL_MODEL[1])
        self.motors.add_dynamixel('revB_41', 11, DYNAMIXEL_MODEL[1])
        self.motors.add_dynamixel('rev_42', 12, DYNAMIXEL_MODEL[0])

        self.motors.init()
        if not self.motors.ping_all():
            raise BaseException("motors aren't configured correctly")


    def setup_motors(self):
        # set operating mode to position control
        self.motors.for_all(lambda motor: motor.set_operating_mode(3))

        for motor in self.motors:
            # sequentially turn on leds
            motor.set_led(True)
            print(f"Motor n°{motor.id} ... Ready!")
            # print("Name : ", motor.name)
            time.sleep(0.3)
        # enable motor control
        self.motors.enable_all()
        self.motors.for_all(lambda motor: motor.set_profile_velocity(150)) # 262




    def degree_to_tic(self, value):
        """from degree to tic, with offset of 180°"""
        return int((value + np.pi) / (2*np.pi) * 4096)

    def tic_to_degree(self, tic):
        """from tic to degree, with offset of 180°"""
        return float((tic - 2048) / 4096 * 2 * np.pi)
        # return int((value + np.pi) / (2*np.pi) * 4096)


    def bound_joints_limit(self, tic, motor):
        """clip out-of-bounds commands"""
        min, max = self.joints_limits[motor.name]
        if tic > max:
            return max
        if tic < min:
            return min
        return tic




    def go_to(self, goal_pos : dict):
        for motor in self.motors:
            value = goal_pos[motor.name]
            tic = self.degree_to_tic(value)
            tic = self.bound_joints_limit(tic, motor)
            motor.set_goal_position(tic)
        # self.motors.for_all(lambda motor: motor.set_goal_position(self.degree_to_tic(goal_pos[motor.name])))


    def get_motors_pos(self):
        pos = []
        motors_names = ["revA_11", "revB_11","rev_12","revA_21", "revB_21","rev_22","revA_31", "revB_31", "rev_32", "revA_41", "revB_41", "rev_42"]
        for k in range(len(motors_names)):
            tic = self.motors[motors_names[k]].get_present_position()
            pos.append(self.tic_to_degree(tic))
        return pos


    def get_motors_load(self):
        Load = []
        motors_names = ["revA_11", "revB_11","rev_12","revA_21", "revB_21","rev_22","revA_31", "revB_31", "rev_32", "revA_41", "revB_41", "rev_42"]

        for k in range(len(motors_names)):
            load = self.motors[motors_names[k]].get_present_load()

            if load < 1000:
                Load.append(float(load/1000))
            elif load > 1000:
                Load.append(float((65535 - load)/1000) * 1.5)

            # Load.append(float(load))
        # print(Load)
        return Load 

        


    def shutdown(self):
        # shut down
        self.motors.disable_all()
        self.motors.for_all(lambda motor: motor.set_led(False))



    def disable_overload_shutdown():
        self.motors.for_all(lambda motor: motor.set_shutdown(20))


    def get_shutdown(self):
        shut_msgs = []
        motors_names = ["revA_11", "revB_11","rev_12","revA_21", "revB_21","rev_22","revA_31", "revB_31", "rev_32", "revA_41", "revB_41", "rev_42"]

        for k in range(len(motors_names)):
            shut = self.motors[motors_names[k]].get_hardware_error_status()

            if shut != 0:
                shut_msgs.append(motors_names[k])
                self.motors[motors_names[k]].reboot()

        return shut_msgs 

"""


class OneLeggedSpider:
    def __init__(self):

        self.motors = DynamixelManager(baud_rate=115200)
        self.motors.add_dynamixel('A', 1, DYNAMIXEL_MODEL[1])
        self.motors.add_dynamixel('B', 2, DYNAMIXEL_MODEL[1])
        self.motors.add_dynamixel('C', 3, DYNAMIXEL_MODEL[0])

        self.motors.init()
        if not self.motors.ping_all():
            raise BaseException("motors aren't configured correctly")


    def setup_motors(self):
        # set operating mode to position control
        self.motors.for_all(lambda motor: motor.set_operating_mode(3))

        for motor in self.motors:
            # sequentially turn on leds
            motor.set_led(True)
            time.sleep(0.2)
        # enable motor control
        self.motors.enable_all()
        self.motors.for_all(lambda motor: motor.set_profile_velocity(100)) # 262



    def go_to(self, goal_pos):
        self.motors.for_all(lambda motor: motor.set_goal_position(self.degree_to_tic(goal_pos[motor.name])))


    def shutdown(self):
        # shut down
        self.motors.disable_all()
        self.motors.for_all(lambda motor: motor.set_led(False))

"""