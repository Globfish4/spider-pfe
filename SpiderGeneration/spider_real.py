from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.utils.stage import create_new_stage
from isaacsim.core.api import World
from isaacsim.core.utils.viewports import set_camera_view

from pxr import Gf, Sdf, UsdLux, UsdPhysics, PhysxSchema, UsdGeom

from isaacsim.core.prims import SingleXFormPrim, SingleGeometryPrim
from isaacsim.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
from isaacsim.core.api.objects import DynamicCylinder, DynamicCuboid
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.api.controllers import ArticulationController


from dynam_2XL import Dynam2XLSim, initialize_world
from dynam_XL import DynamXLSim



def print_dir(obj):
    print("################################################################\n")
    try:
        print("####### ATTRIBUTES")
        print(obj.GetAttributes())
    except:
        print("GetAttributes function didn't work")
    
    try:
        print("####### DIR")
        print(dir(obj))
    except:
        pass
    print("################################################################\n")





def get_quatf(angles):
    """
    angles: ndArray
    returns: Gf.Quatf quaternion value 
    """
    Q = euler_angles_to_quat(angles, degrees=True)
    return Gf.Quatf(Q[0], Q[1], Q[2], Q[3])

def get_quatd(angles):
    """
    angles: ndArray
    returns: Gf.Quatf quaternion value 
    """
    Q = euler_angles_to_quat(angles, degrees=True)
    return Gf.Quatd(Q[0], Q[1], Q[2], Q[3])






class DynamSpider():
    def __init__(self, world: World):
        self.world = world

        self.create_prims()
        self.add_servos()
        self.attach_servos()
        self.setup_articulation()

        self.world.reset()

        self.state = "LAYING"



    def create_prims(self):
        # Create robot's XForm with articulation root
        self.servo_xform = SingleXFormPrim(prim_path="/Spider",
                            name="Spider",
                            position=Gf.Vec3d(0.0, 0.0, 0.0))
        self.world.stage.SetDefaultPrim(self.servo_xform.prim)
        rootT = UsdPhysics.ArticulationRootAPI(self.servo_xform.prim).Apply(self.servo_xform.prim)

        # Create bodies
        self._body0 = DynamicCuboid(prim_path="/Spider/Abdomen",
                            position=np.array([0, 0, 0.009]),
                            scale=np.array([0.15, 0.15, 0.018]), 
                            mass=2)
        

        self._body11 = DynamicCylinder(prim_path="/Spider/Paw11",
                            position=np.array([0.215, 0, 0.018]), 
                            orientation=euler_angles_to_quat(np.array([0, np.pi/2, 0])),
                            radius=0.01, 
                            height=0.1, 
                            mass=0.01)
        self._body12 = DynamicCylinder(prim_path="/Spider/Paw12",
                            position=np.array([0.385, 0, 0.018]), 
                            orientation=euler_angles_to_quat(np.array([0, np.pi/2, 0])),
                            radius=0.01, 
                            height=0.1, 
                            mass=0.01)
        
        self._body21 = DynamicCylinder(prim_path="/Spider/Paw21",
                            position=np.array([0, 0.215, 0.018]), 
                            orientation=euler_angles_to_quat(np.array([np.pi/2, 0, 0])),
                            radius=0.01, 
                            height=0.1, 
                            mass=0.01)
        self._body22 = DynamicCylinder(prim_path="/Spider/Paw22",
                            position=np.array([0, 0.385, 0.018]), 
                            orientation=euler_angles_to_quat(np.array([np.pi/2, 0, 0])),
                            radius=0.01, 
                            height=0.1, 
                            mass=0.01)
        
        self._body31 = DynamicCylinder(prim_path="/Spider/Paw31",
                            position=np.array([-0.215, 0, 0.018]), 
                            orientation=euler_angles_to_quat(np.array([0, -np.pi/2, 0])),
                            radius=0.01, 
                            height=0.1, 
                            mass=0.01)
        self._body32 = DynamicCylinder(prim_path="/Spider/Paw32",
                            position=np.array([-0.385, 0, 0.018]), 
                            orientation=euler_angles_to_quat(np.array([0, -np.pi/2, 0])),
                            radius=0.01, 
                            height=0.1, 
                            mass=0.01)
        
        self._body41 = DynamicCylinder(prim_path="/Spider/Paw41",
                            position=np.array([0, -0.215, 0.018]), 
                            orientation=euler_angles_to_quat(np.array([-np.pi/2, 0, 0])),
                            radius=0.01, 
                            height=0.1, 
                            mass=0.01)
        self._body42 = DynamicCylinder(prim_path="/Spider/Paw42",
                            position=np.array([0, -0.385, 0.018]), 
                            orientation=euler_angles_to_quat(np.array([-np.pi/2, 0, 0])),
                            radius=0.01, 
                            height=0.1, 
                            mass=0.01)






    def add_servos(self):
        self.servo11 = Dynam2XLSim(self.world, np.array([0.145, 0, 0.018]), np.array([0, -np.pi/2, 0]), name="Spider/Servo11", id="11")
        self.servo21 = Dynam2XLSim(self.world, np.array([0, 0.145, 0.018]), np.array([0, -np.pi/2, np.pi/2]), name="Spider/Servo21", id="21")
        self.servo31 = Dynam2XLSim(self.world, np.array([-0.145, 0, 0.018]), np.array([0, -np.pi/2, np.pi/2*2]), name="Spider/Servo31", id="31")
        self.servo41 = Dynam2XLSim(self.world, np.array([0, -0.145, 0.018]), np.array([0, -np.pi/2, np.pi/2*3]), name="Spider/Servo41", id="41")
        
        self.servo12 = DynamXLSim(self.world, np.array([0.335, 0, 0.018]), np.array([np.pi/2, 0, -np.pi/2]), name="Spider/Servo12", id="12")
        self.servo22 = DynamXLSim(self.world, np.array([0, 0.335, 0.018]), np.array([np.pi/2, 0, 0]), name="Spider/Servo22", id="22")
        self.servo32 = DynamXLSim(self.world, np.array([-0.335, 0, 0.018]), np.array([np.pi/2, 0, np.pi/2]), name="Spider/Servo32", id="32")
        self.servo42 = DynamXLSim(self.world, np.array([0, -0.335, 0.018]), np.array([np.pi/2, 0, -np.pi]), name="Spider/Servo42", id="42")

        self.servo11.set_joint_lower_upper_limits(-50, 50, -40, 40)
        self.servo21.set_joint_lower_upper_limits(-50, 50, -40, 40)
        self.servo31.set_joint_lower_upper_limits(-50, 50, -40, 40)
        self.servo41.set_joint_lower_upper_limits(-50, 50, -40, 40)
        self.servo12.set_joint_lower_upper_limits(30, 100)
        self.servo22.set_joint_lower_upper_limits(30, 100)
        self.servo32.set_joint_lower_upper_limits(30, 100)
        self.servo42.set_joint_lower_upper_limits(30, 100)
    


    def attach_servos(self):
        self.servo11.attach_body_to_rotor(prim_path1="/Spider/Abdomen", prim_path2="/Spider/Paw11", 
                                          localpos0A=Gf.Vec3f(0.72, 0.0, 1.58333), localrot0A=np.array([0, 0, 0]),
                                          localpos0B=Gf.Vec3f(0.0, 0.0195, -0.083), localrot0B=np.array([-90, 0, 180]))
        self.servo21.attach_body_to_rotor(prim_path1="/Spider/Abdomen", prim_path2="/Spider/Paw21", 
                                          localpos0A=Gf.Vec3f(0.0, 0.72, 1.58333), localrot0A=np.array([0, 0, 90]),
                                          localpos0B=Gf.Vec3f(-0.0195, 0.0, 0.083), localrot0B=np.array([90, 0, 90]))
        self.servo31.attach_body_to_rotor(prim_path1="/Spider/Abdomen", prim_path2="/Spider/Paw31", 
                                          localpos0A=Gf.Vec3f(-0.72, 0.0, 1.58333), localrot0A=np.array([0, 0, 180]),
                                          localpos0B=Gf.Vec3f(0.0, -0.0195, -0.083), localrot0B=np.array([-90, 0, 0]))
        self.servo41.attach_body_to_rotor(prim_path1="/Spider/Abdomen", prim_path2="/Spider/Paw41", 
                                          localpos0A=Gf.Vec3f(0.0, -0.72, 1.58333), localrot0A=np.array([0, 0, -90]),
                                          localpos0B=Gf.Vec3f(0.0195, 0.0, 0.083), localrot0B=np.array([90, 0, -90]))


        self.servo12.attach_body_to_rotor(prim_path1="/Spider/Paw11", prim_path2="/Spider/Paw12", 
                                          localpos0A=Gf.Vec3f(0.0, -0.02, 0.078), localrot0A=np.array([0, -90, 90]),
                                          localpos0B=Gf.Vec3f(0.0, 0.0, -0.07991), localrot0B=np.array([-180, 0, -90]))
        self.servo22.attach_body_to_rotor(prim_path1="/Spider/Paw21", prim_path2="/Spider/Paw22", 
                                          localpos0A=Gf.Vec3f(0.02, 0.0, -0.078), localrot0A=np.array([0, 90, 0]),
                                          localpos0B=Gf.Vec3f(0.0, 0.0, 0.07991), localrot0B=np.array([0, 0, 0]))
        self.servo32.attach_body_to_rotor(prim_path1="/Spider/Paw31", prim_path2="/Spider/Paw32", 
                                          localpos0A=Gf.Vec3f(0.0, 0.02, 0.078), localrot0A=np.array([-90, -90, 0]),
                                          localpos0B=Gf.Vec3f(0.0, 0.0, -0.07991), localrot0B=np.array([180, 0, -270]))
        self.servo42.attach_body_to_rotor(prim_path1="/Spider/Paw41", prim_path2="/Spider/Paw42", 
                                          localpos0A=Gf.Vec3f(-0.02, 0.0, -0.078), localrot0A=np.array([180, 90, 0]),
                                          localpos0B=Gf.Vec3f(0.0, 0.0, 0.07991), localrot0B=np.array([0, 0, 180]))



    def setup_articulation(self):
        self.world.reset()
        self.art = SingleArticulation(prim_path=f"/Spider")
        self.world.scene.add(self.art)
        self.art.initialize()

        


        print("##########################################################################")
        print("########### TO USE ARTICULATION: ")
        print("########### DOF names: ", self.art.dof_names)






    def act(self, Nsteps, action):
        self.art.apply_action(control_actions=action)
        for _ in range(Nsteps):
            world.step(render=True)

            joo = self.art.get_measured_joint_efforts()
            print("max : ", np.max(np.abs(joo)), "      joint n° : ", np.argmax(joo))



    def stand_up(self):
        print(f"########### State: {self.state} | Standing Up #####################")
        action = ArticulationAction(joint_positions=np.array([0, 0, 0, 0, np.pi/4, np.pi/4, np.pi/4, np.pi/4, 3*np.pi/4, 3*np.pi/4, 3*np.pi/4, 3*np.pi/4]))
        self.act(20, action)
        action = ArticulationAction(joint_positions=np.array([0, 0, 0, 0, -np.pi/8, -np.pi/8, -np.pi/8, -np.pi/8, 3*np.pi/8, 3*np.pi/8, 3*np.pi/8, 3*np.pi/8]))
        self.act(20, action)

        self.state = "STANDING"


    def lay_down(self):
        print(f"########### State: {self.state} | laying down ###################")
        action = ArticulationAction(joint_positions=np.array([0, 0, 0, 0, np.pi/4, np.pi/4, np.pi/4, np.pi/4, 3*np.pi/4, 3*np.pi/4, 3*np.pi/4, 3*np.pi/4]))
        self.act(20, action)
        action = ArticulationAction(joint_positions=np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0]))
        self.act(20, action)

        self.state = "LAYING"


    def turn_around(self, heading):
        eps = 0.2
        _, rot0 = self._body0.get_local_pose()
        rot0Euler = quat_to_euler_angles(rot0)
        print(f"########### State: {self.state} | Rotating to Heading {heading}° ###################")
        print(f"##### Initial heading:", rot0Euler[2])
        action = ArticulationAction(joint_positions=np.array([0, -np.pi/12, 0, -np.pi/12, 
                                                              2*np.pi/12, 2*np.pi/12, 2*np.pi/12, 2*np.pi/12, 
                                                              8*np.pi/12, 8*np.pi/12, 8*np.pi/12, 8*np.pi/12]))
        self.act(20, action) # Take neutral stable pose

        while abs(rot0Euler[2] - heading) > eps:
            action = ArticulationAction(joint_positions=np.array([np.pi/12, -np.pi/12, np.pi/12, -np.pi/12,
                                                                3*np.pi/12, 0, 3*np.pi/12, 0, 
                                                                8*np.pi/12, 8*np.pi/12, 8*np.pi/12, 8*np.pi/12]))
            self.act(10, action) # Raise two opposite legs and rotate them

            action = ArticulationAction(joint_positions=np.array([np.pi/12, -np.pi/12, np.pi/12, -np.pi/12,
                                                                2*np.pi/12, 0, 2*np.pi/12, 0, 
                                                                8*np.pi/12, 8*np.pi/12, 8*np.pi/12, 8*np.pi/12]))
            self.act(10, action) # Put those two legs down

            action = ArticulationAction(joint_positions=np.array([-np.pi/12, np.pi/12, -np.pi/12, np.pi/12,
                                                                0, 3*np.pi/12, 0, 3*np.pi/12, 
                                                                8*np.pi/12, 8*np.pi/12, 8*np.pi/12, 8*np.pi/12]))
            self.act(10, action) # Rotate body while raising the two other opposite legs

            action = ArticulationAction(joint_positions=np.array([-np.pi/12, np.pi/12, -np.pi/12, np.pi/12,
                                                                0, 2*np.pi/12, 0, 2*np.pi/12, 
                                                                8*np.pi/12, 8*np.pi/12, 8*np.pi/12, 8*np.pi/12]))
            self.act(10, action) # put down the two legs

            # Compute walked distance
            _, rot0 = self._body0.get_local_pose()
            rot0Euler = quat_to_euler_angles(rot0)
            print("##### Current Heading: ", rot0Euler[2])

        action = ArticulationAction(joint_positions=np.array([0, 0, 0, 0, 
                                                              2*np.pi/12, 2*np.pi/12, 2*np.pi/12, 2*np.pi/12, 
                                                              8*np.pi/12, 8*np.pi/12, 8*np.pi/12, 8*np.pi/12]))
        self.act(20, action) # Take neutral stable pose

     


    def walk(self, distance):
        pos0, _ = self._body0.get_local_pose()
        dist = 0 # current walked distance
        print(f"########### State: {self.state} | Walking {distance}m ###################")
        print(f"##### Initial Pos: {pos0}")
        action = ArticulationAction(joint_positions=np.array([0, np.pi/12, 0, -np.pi/12, 
                                                              2*np.pi/12, 2*np.pi/12, 2*np.pi/12, 2*np.pi/12, 
                                                              8*np.pi/12, 8*np.pi/12, 8*np.pi/12, 8*np.pi/12]))
        self.act(20, action) # Take neutral stable pose


        while dist < distance:
            action = ArticulationAction(joint_positions=np.array([np.pi/12, np.pi/12, -np.pi/12, -np.pi/12,
                                                                3*np.pi/12, 0, 3*np.pi/12, 0, 
                                                                8*np.pi/12, 8*np.pi/12, 8*np.pi/12, 8*np.pi/12]))
            self.act(7, action) # Raise two opposite legs and rotate them in the same direction

            action = ArticulationAction(joint_positions=np.array([np.pi/12, np.pi/12, -np.pi/12, -np.pi/12,
                                                                2*np.pi/12, 0, 2*np.pi/12, 0, 
                                                                8*np.pi/12, 8*np.pi/12, 8*np.pi/12, 8*np.pi/12]))
            self.act(7, action) # Put those two legs down

            action = ArticulationAction(joint_positions=np.array([-np.pi/12, -np.pi/12, np.pi/12, np.pi/12,
                                                                0, 3*np.pi/12, 0, 3*np.pi/12, 
                                                                8*np.pi/12, 8*np.pi/12, 8*np.pi/12, 8*np.pi/12]))
            self.act(7, action) # Rotate body while raising the two other opposite legs

            action = ArticulationAction(joint_positions=np.array([-np.pi/12, -np.pi/12, np.pi/12, np.pi/12,
                                                                0, 2*np.pi/12, 0, 2*np.pi/12, 
                                                                8*np.pi/12, 8*np.pi/12, 8*np.pi/12, 8*np.pi/12]))
            self.act(7, action) # put down the two legs

            # Compute walked distance
            pos1, _ = self._body0.get_local_pose()
            dist = np.linalg.norm(pos0 - pos1)
            # print("##### Walked Distance: ", dist)
            # print("##### Current Pos: ", pos1)
            # print("## Joint efforts : ", self.art.get_measured_joint_efforts())

        action = ArticulationAction(joint_positions=np.array([0, 0, 0, 0, 
                                                              2*np.pi/12, 2*np.pi/12, 2*np.pi/12, 2*np.pi/12, 
                                                              8*np.pi/12, 8*np.pi/12, 8*np.pi/12, 8*np.pi/12]))
        self.act(20, action) # Take neutral stable pose








def neutral_simulation(n):
    print("########### Starting Neutral Simulation #####################")
    for _ in range(n):
        # print("################################################## Step N°" + str(_))
        world.step(render=True)



def ui(spider: DynamSpider):
    while True:
        cmd = input("$ ")
        if cmd[:4] == "walk":
            spider.walk(eval(cmd[5:]))
        if cmd[:4] == "turn":
            spider.turn_around(eval(cmd[5:]))
        if cmd[:5] == "stand":
            spider.stand_up()
        if cmd[:3] == "lay":
            spider.lay_down()
        if cmd[:4] == "quit":
            return 0





if __name__ == "__main__":
    world = initialize_world()
    spider = DynamSpider(world)

    neutral_simulation(200)
    spider.stand_up()
    neutral_simulation(10)
    spider.lay_down()
    neutral_simulation(10)
    spider.walk(1)
    neutral_simulation(10)
    spider.turn_around(3*np.pi/4)
    
    # neutral_simulation(10)
    # spider.walk(1)
    neutral_simulation(500)
    # ui(spider)

    neutral_simulation(99999999)
    simulation_app.close()



