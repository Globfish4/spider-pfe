
# Imports
from isaacsim.simulation_app import SimulationApp
if __name__ == "__main__":
    simulation_app = SimulationApp({"headless": False})


import numpy as np
from isaacsim.core.utils.stage import create_new_stage
from isaacsim.core.api import World
from isaacsim.core.utils.viewports import set_camera_view
from pxr import Gf, Sdf, UsdLux, UsdPhysics

from isaacsim.core.prims import SingleXFormPrim, SingleGeometryPrim
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.api.objects import DynamicCylinder, DynamicCuboid
from isaacsim.core.api.materials import OmniPBR
from isaacsim.core.utils.prims import create_prim, add_reference_to_stage






def get_quatf(angles):
    """
    angles: ndArray
    returns: Gf.Quatf quaternion value 
    """
    Q = euler_angles_to_quat(angles, degrees=True)
    return Gf.Quatf(Q[0], Q[1], Q[2], Q[3])




class DynamXLSim():
    def __init__(self, world, pos=np.array([0.0, 0.0, 0.0]), orient=np.array([0, 0, 0]), name="Servo", id=""):
        self.pos = pos
        self.orient = orient
        self.world = world
        self.name = name
        self.id = id

        self.create_prims()
        self.load_mesh()
        self.add_joint()




    def create_prims(self):
        # Create robot's XForm with articulation root
        self.servo_xform = SingleXFormPrim(prim_path=f"/{self.name}",
                            name=self.name)

        # Create bodies
        self._body0 = DynamicCuboid(prim_path=f"/{self.name}/Body0_{self.id}",
                                   position=np.array([0, 0, 0.03]),
                                   scale=np.array([0.034, 0.0285, 0.0465]), 
                                   mass=0.075) # mass = 0.075 kg
        self._body1 = DynamicCylinder(prim_path=f"/{self.name}/Body1_{self.id}",
                                   position=np.array([0.02, 0, 0.042]), 
                                   orientation=euler_angles_to_quat(np.array([np.pi/2, np.pi/2, np.pi/2])),
                                   radius=0.01, 
                                   height=0.003, 
                                   mass=0.007)
        
        self.servo_xform.set_local_pose(translation=self.pos, orientation=euler_angles_to_quat(self.orient))

        # Make invisible
        mat = OmniPBR(prim_path=f"/{self.name}/Looks/pbr", 
                            color=np.array([0.0, 0.0, 1.0]))
        shad = mat.shaders_list[0]
        shad.CreateInput("opacity_constant", Sdf.ValueTypeNames.Float).Set(0.0)
        shad.CreateInput("enable_opacity", Sdf.ValueTypeNames.Bool).Set(True)

        self._body0.apply_visual_material(mat, weaker_than_descendants=True)
        self._body1.apply_visual_material(mat, weaker_than_descendants=True)




    def load_mesh(self):
        # Load visible meshes of dynamixel XL430
        add_reference_to_stage(usd_path="CAMILO/dynamixel/dynam_XM/rotorless.usdc", 
                               prim_path=f"/{self.name}/Body0_{self.id}/rotorless")
        self._rotorless = SingleGeometryPrim(
            prim_path=f"/{self.name}/Body0_{self.id}/rotorless", 
            translation=[0, 0.017, -0.5], 
            orientation=euler_angles_to_quat(np.array([0, 0, np.pi/2])))
        wscale = self._rotorless.get_world_scale()
        self._rotorless.set_local_scale(scale=[1/wscale[0], 1/wscale[1], 1/wscale[2]])


        add_reference_to_stage(usd_path="CAMILO/dynamixel/dynam_XM/rotoronly.usdc", 
                               prim_path=f"/{self.name}/Body1_{self.id}/rotoronly")
        self._rotoronly = SingleGeometryPrim(
            prim_path=f"/{self.name}/Body1_{self.id}/rotoronly", 
            translation=[0.0005, -0.0353, -0.02], 
            orientation=euler_angles_to_quat(np.array([-np.pi/2, 0, 0])))
        wscale = self._rotoronly.get_world_scale()
        self._rotoronly.set_local_scale(scale=[1/wscale[0], 1/wscale[1], 1/wscale[2]])

        # self.world.reset()



    def add_joint(self):
        self.rev_prim = create_prim(prim_path=f"/{self.name}/Body1_{self.id}/rev_{self.id}", prim_type="PhysicsRevoluteJoint")
        self.rev_joint = UsdPhysics.RevoluteJoint(self.rev_prim)

        self.rev_joint.CreateBody0Rel().SetTargets([Sdf.Path(f"/{self.name}/Body0_{self.id}")])
        self.rev_joint.CreateBody1Rel().SetTargets([Sdf.Path(f"/{self.name}/Body1_{self.id}")])
        self.rev_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(1.32353, 0.0, 0.25806))
        self.rev_joint.CreateLocalRot0Attr().Set(get_quatf(np.array([0, 0, 0])))
        self.rev_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0.025))
        self.rev_joint.CreateLocalRot1Attr().Set(get_quatf(np.array([0, -90, 0])))
        self.rev_joint.CreateAxisAttr().Set("X")

        self.drive_rev = UsdPhysics.DriveAPI(self.rev_prim, "angular")
        self.drive_rev.Apply(self.rev_prim, "angular")
        self.drive_rev.CreateTypeAttr().Set("force")
        self.drive_rev.CreateMaxForceAttr().Set(200.0)
        self.drive_rev.CreateStiffnessAttr().Set(125)
        self.drive_rev.CreateDampingAttr().Set(40)




    def attach_body_to_rotor(self, 
                             prim_path1: str | None = None, 
                             prim_path2: str | None = None, 
                             localpos0A=Gf.Vec3f(0, 0, 0), 
                             localrot0A=np.array([0, 0, 0]),
                             localpos0B=Gf.Vec3f(0, 0, 0), 
                             localrot0B=np.array([0, 0, 0])):
        if prim_path1 is not None:
            self.fix_prim1 = self.world.stage.DefinePrim(f"/{self.name}/Body1_{self.id}/fixA_{self.id}", "PhysicsFixedJoint")
            self.fix_joint1 = UsdPhysics.FixedJoint(self.fix_prim1)
            self.fix_joint1.CreateBody0Rel().SetTargets([Sdf.Path(prim_path1)])
            self.fix_joint1.CreateBody1Rel().SetTargets([Sdf.Path(f"/{self.name}/Body1_{self.id}")])
            self.fix_joint1.CreateLocalPos0Attr().Set(localpos0A)
            self.fix_joint1.CreateLocalRot0Attr().Set(get_quatf(localrot0A))

        if prim_path2 is not None:
            self.fix_prim2 = self.world.stage.DefinePrim(f"/{self.name}/Body0_{self.id}/fixB_{self.id}", "PhysicsFixedJoint")
            self.fix_joint2 = UsdPhysics.RevoluteJoint(self.fix_prim2)
            self.fix_joint2.CreateBody0Rel().SetTargets([Sdf.Path(prim_path2)])
            self.fix_joint2.CreateBody1Rel().SetTargets([Sdf.Path(f"/{self.name}/Body0_{self.id}")])
            self.fix_joint2.CreateLocalPos0Attr().Set(localpos0B)
            self.fix_joint2.CreateLocalRot0Attr().Set(get_quatf(localrot0B))


    def set_joint_lower_upper_limits(self, low_lim, up_lim):
        self.rev_joint.CreateLowerLimitAttr().Set(low_lim)
        self.rev_joint.CreateUpperLimitAttr().Set(up_lim)



    # def setup_articulation(self):
    #     self.world.reset()
    #     self.effort_sensor = EffortSensor(prim_path=f"/{self.name}/Body1/rev")
    #     # self.effort_sensor.initialize()
    #     self.art = SingleArticulation(prim_path=f"/{self.name}")                 
    #     self.art.initialize()



    # def set_wanted_action(self, act=np.array([0, 0])):
    #     """
    #     pos in deg, vel in deg/s
    #     """
    #     self.targetPos = np.deg2rad(act[0])
    #     self.targetVel = np.deg2rad(act[1])






    # def compute_effort_PD(self):
    #     pos = self.art.get_joint_positions()[0]
    #     vit = self.art.get_joint_velocities()[0]

    #     # print(pos, vit, self.position_d, self.velocity_d)
    #     effort = self.stiff * (self.targetPos - pos) + self.damp * (self.targetVel - vit)
    #     effort = np.min([abs(effort), 1]) * effort / abs(effort)
    #     # print("Computed effort: ", effort)
    #     return effort




    # def process_integral(self, erreur):
    #     self.integrale.append(erreur)
    #     if len(self.integrale) > 1000:
    #         self.integrale = self.integrale[1:]

    # def compute_effort_PID(self):
    #     pos = self.art.get_joint_positions()[0]
    #     vit = self.art.get_joint_velocities()[0]
    #     erreur = self.targetPos - pos
    #     self.process_integral(erreur)
    #     effort = self.stiff * erreur + self.damp * (self.targetVel - vit) + self.ki * sum(self.integrale)
    #     # print("computed effort : ", effort)
    #     effort = np.min([abs(effort), 20]) * effort / abs(effort)
    #     return effort






    # def apply_effort_action(self, event):
    #     effort = self.compute_effort_PD()
    #     # effort = self.compute_effort_PID()
    #     self.art.set_joint_efforts(effort)







def initialize_world():
    create_new_stage()
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    world.reset()
    world.get_physics_context().enable_gpu_dynamics(True)
    set_camera_view([0.8, 0.3, 0.15], [0.0, 0.0, 0.05])

    sphereLight = UsdLux.SphereLight.Define(world.stage, Sdf.Path("/World/MySphereLight"))
    sphereLight.CreateIntensityAttr(50000.0)
    sphereLight.AddTranslateOp().Set(Gf.Vec3f(3.0, 0, 1.5))

    world.reset()
    return world




def neutral_simulation():
    print("################ Starting Neutral Simulation ################")

    for _ in range(100000):
        # print("################################################## Step N°" + str(_))
        world.step(render=True)


        # if _ > 120:
        #     try:
        #         effort = dy.compute_effort_PD()
        #         dy.art.set_joint_efforts(np.array([effort]))
        #         print("Applied effort : ", dy.art.get_applied_joint_efforts())
        #         print("Measured effort : ", dy.art.get_measured_joint_efforts())
        #     except:
        #         pass







if __name__ == "__main__":

    world = initialize_world()

    stiff = 0.00005
    damp = 0.00001
    ki = 0.000001
    dy = DynamXLSim(world, pos=np.array([0.5, 0, 0.3]), orient=np.array([-np.pi/2, 0, np.pi/2]))
    # dy.set_wanted_action(np.array([90, 0]))

    # cube = DynamicCuboid(prim_path="/World/cube", 
                        #  position=np.array([0, 0.02, 0.07]), 
                        #  scale=np.array([0.02, 0.02, 0.02]), 
                        #  mass=0.01)
    # dy.attach_to_servo_rotor(cube.prim_path)

    # world.step(render=True)
    # world.step(render=True)
    # world.reset()

    # dy.art.initialize()
    # dy.effort_sensor.initialize()
    # world.add_render_callback("pos_vel_regulator", dy.apply_effort_action)

    neutral_simulation()
    simulation_app.close()

