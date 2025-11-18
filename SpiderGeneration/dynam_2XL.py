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
from isaacsim.core.api.materials import OmniPBR, PhysicsMaterial
from isaacsim.core.utils.prims import create_prim, add_reference_to_stage






def get_quatf(angles):
    """
    angles: ndArray
    returns: Gf.Quatf quaternion value 
    """
    Q = euler_angles_to_quat(angles, degrees=True)
    return Gf.Quatf(Q[0], Q[1], Q[2], Q[3])




class Dynam2XLSim():
    def __init__(self, world, pos=np.array([0.0, 0.0, 0.0]), orient=np.array([0, 0, 0]), name="Servo", id=""):
        self.pos = pos
        self.orient = orient
        self.world = world
        self.name = name
        self.id = id

        self.create_prims()
        self.load_mesh()
        self.add_joints()



    def create_prims(self):
        # Create robot's XForm with articulation root
        self.servo_xform = SingleXFormPrim(prim_path=f"/{self.name}",
                            name=self.name)

        # Create bodies
        self._body0 = DynamicCuboid(prim_path=f"/{self.name}/Body0_{self.id}",
                            position=np.array([0, 0, 0.025]),
                            scale=np.array([0.036, 0.036, 0.0465]), 
                            mass=0.0982) # mass = 0.0982 kg
        self._body1 = DynamicCylinder(prim_path=f"/{self.name}/Body1_{self.id}",
                            position=np.array([0.0195, 0, 0.037]), 
                            orientation=euler_angles_to_quat(np.array([0, np.pi/2, 0])),
                            radius=0.01, 
                            height=0.003, 
                            mass=0.007)
        self._body2 = DynamicCylinder(prim_path=f"/{self.name}/Body2_{self.id}",
                           position=np.array([0, 0.0195, 0.013]), 
                           orientation=euler_angles_to_quat(np.array([np.pi/2, 0, 0])),
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
        self._body2.apply_visual_material(mat, weaker_than_descendants=True)




    def load_mesh(self):
        # Load visible meshes of dynamixel XL430
        add_reference_to_stage(usd_path="CAMILO/dynamixel/dynam_2XL/2XL.usdc", 
                               prim_path=f"/{self.name}/Body0_{self.id}/rotorless")
        self._rotorless = SingleGeometryPrim(
            prim_path=f"/{self.name}/Body0_{self.id}/rotorless", 
            translation=np.array([0, 0, -0.5]), 
            orientation=euler_angles_to_quat(np.array([0.0, 0.0, 0.0])))
        wscale = self._rotorless.get_world_scale()
        self._rotorless.set_local_scale(scale=[1/wscale[0], 1/wscale[1], 1/wscale[2]])

        add_reference_to_stage(usd_path="CAMILO/dynamixel/dynam_2XL/XLA.usdc", 
                               prim_path=f"/{self.name}/Body1_{self.id}/rotoronly")
        self._rotoronlyA = SingleGeometryPrim(
            prim_path=f"/{self.name}/Body1_{self.id}/rotoronly", 
            translation=np.array([0, 0, -0.00375]), 
            orientation=euler_angles_to_quat(np.array([0, -np.pi/2, 0])))
        wscale = self._rotoronlyA.get_world_scale()
        self._rotoronlyA.set_local_scale(scale=[1/wscale[0], 1/wscale[1], 1/wscale[2]])

        add_reference_to_stage(usd_path="CAMILO/dynamixel/dynam_2XL/XLB.usdc", 
                               prim_path=f"/{self.name}/Body2_{self.id}/rotoronly")
        self._rotoronlyB = SingleGeometryPrim(
            prim_path=f"/{self.name}/Body2_{self.id}/rotoronly", 
            translation=np.array([0, 0, 0.00375]), 
            orientation=euler_angles_to_quat(np.array([-np.pi/2, 0, 0])))
        wscale = self._rotoronlyB.get_world_scale()
        self._rotoronlyB.set_local_scale(scale=[1/wscale[0], 1/wscale[1], 1/wscale[2]])




    def add_joints(self):
        self.rev_primA = create_prim(prim_path=f"/{self.name}/Body1_{self.id}/revA_{self.id}", prim_type="PhysicsRevoluteJoint")
        self.rev_jointA = UsdPhysics.RevoluteJoint(self.rev_primA)
        self.rev_jointA.CreateBody0Rel().SetTargets([Sdf.Path(f"/{self.name}/Body0_{self.id}")])
        self.rev_jointA.CreateBody1Rel().SetTargets([Sdf.Path(f"/{self.name}/Body1_{self.id}")])
        self.rev_jointA.CreateLocalPos0Attr().Set(Gf.Vec3f(0.54167, 0, 0.25806))
        self.rev_jointA.CreateLocalRot0Attr().Set(get_quatf(np.array([0, 90, 0])))
        self.rev_jointA.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        self.rev_jointA.CreateLocalRot1Attr().Set(get_quatf(np.array([0, 0, 0])))
        self.rev_jointA.CreateAxisAttr().Set("Z")
        self.drive_revA = UsdPhysics.DriveAPI(self.rev_primA, "angular")
        self.drive_revA.Apply(self.rev_primA, "angular")
        self.drive_revA.CreateTypeAttr().Set("force")
        self.drive_revA.CreateMaxForceAttr().Set(100.0)
        self.drive_revA.CreateStiffnessAttr().Set(1000)
        self.drive_revA.CreateDampingAttr().Set(100)

        self.rev_primB = create_prim(prim_path=f"/{self.name}/Body2_{self.id}/revB_{self.id}", prim_type="PhysicsRevoluteJoint")
        self.rev_jointB = UsdPhysics.RevoluteJoint(self.rev_primB)
        self.rev_jointB.CreateBody0Rel().SetTargets([Sdf.Path(f"/{self.name}/Body0_{self.id}")])
        self.rev_jointB.CreateBody1Rel().SetTargets([Sdf.Path(f"/{self.name}/Body2_{self.id}")])
        self.rev_jointB.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0.54167, -0.25806))
        self.rev_jointB.CreateLocalRot0Attr().Set(get_quatf(np.array([90, 0, 0])))
        self.rev_jointB.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        self.rev_jointB.CreateLocalRot1Attr().Set(get_quatf(np.array([0, 0, 0])))
        self.rev_jointB.CreateAxisAttr().Set("Z")
        self.drive_revB = UsdPhysics.DriveAPI(self.rev_primB, "angular")
        self.drive_revB.Apply(self.rev_primB, "angular")
        self.drive_revB.CreateTypeAttr().Set("force")
        self.drive_revB.CreateMaxForceAttr().Set(100.0)
        self.drive_revB.CreateStiffnessAttr().Set(1000)
        self.drive_revB.CreateDampingAttr().Set(100)




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
            self.fix_prim2 = self.world.stage.DefinePrim(f"/{self.name}/Body2_{self.id}/fixB_{self.id}", "PhysicsFixedJoint")
            self.fix_joint2 = UsdPhysics.RevoluteJoint(self.fix_prim2)
            self.fix_joint2.CreateBody0Rel().SetTargets([Sdf.Path(prim_path2)])
            self.fix_joint2.CreateBody1Rel().SetTargets([Sdf.Path(f"/{self.name}/Body2_{self.id}")])
            self.fix_joint2.CreateLocalPos0Attr().Set(localpos0B)
            self.fix_joint2.CreateLocalRot0Attr().Set(get_quatf(localrot0B))




    def set_joint_lower_upper_limits(self, low_lim_A, up_lim_A, low_lim_B, up_lim_B):
        self.rev_jointA.CreateLowerLimitAttr().Set(low_lim_A)
        self.rev_jointA.CreateUpperLimitAttr().Set(up_lim_A)
        self.rev_jointB.CreateLowerLimitAttr().Set(low_lim_B)
        self.rev_jointB.CreateUpperLimitAttr().Set(up_lim_B)




def initialize_world():
    create_new_stage()
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()




    physics_material_1 = PhysicsMaterial(
        prim_path="/World/Physics_Materials/ground_plane_material", dynamic_friction=0.1, static_friction=0.1, restitution=0.0)
    # cube.apply_physics_material(physics_material=physics_material_1)


    # --- Attacher le matériau au ground plane ---
    ground_prim = world.stage.GetPrimAtPath("/World/defaultGroundPlane/GroundPlane/CollisionPlane")
    if ground_prim:
        ground_prim.GetRelationship("physics:material:binding").SetTargets([Sdf.Path("/World/Physics_Materials/ground_plane_material")])
        # ground_prim.ApplyAPI(physics_material_1)


    world.reset()
    world.get_physics_context().enable_gpu_dynamics(True)
    set_camera_view([0.8, 0.3, 0.15], [0.0, 0.0, 0.05])

    sphereLight = UsdLux.SphereLight.Define(world.stage, Sdf.Path("/World/MySphereLight"))
    sphereLight.CreateIntensityAttr(50000.0)
    sphereLight.AddTranslateOp().Set(Gf.Vec3f(3.0, 0, 1.5))

    world.reset()
    return world






# from omni.isaac.core import World
# from omni.isaac.core.utils.stage import create_new_stage
# from pxr import UsdLux, UsdPhysics, Sdf, Gf

# def initialize_world():
#     create_new_stage()
#     world = World(stage_units_in_meters=1.0)
#     world.scene.add_default_ground_plane()

#     # --- Créer un PhysicsMaterial USD ---
#     material_path = "/World/Physics_Materials/ground_plane_material"
#     physics_material = UsdPhysics.MaterialAPI.Define(world.stage, Sdf.Path(material_path))
#     physics_material.CreateStaticFrictionAttr(0.1)
#     physics_material.CreateDynamicFrictionAttr(0.1)
#     physics_material.CreateRestitutionAttr(0.0)

#     # --- Attacher le matériau au CollisionPlane ---
#     ground_prim = world.stage.GetPrimAtPath("/World/defaultGroundPlane/GroundPlane/CollisionPlane")
#     if ground_prim.IsValid():
#         ground_prim.GetRelationship("physics:material:binding").SetTargets([Sdf.Path(material_path)])

#     # --- reste inchangé ---
#     world.reset()
#     world.get_physics_context().enable_gpu_dynamics(True)
#     set_camera_view([0.8, 0.3, 0.15], [0.0, 0.0, 0.05])

#     sphereLight = UsdLux.SphereLight.Define(world.stage, Sdf.Path("/World/MySphereLight"))
#     sphereLight.CreateIntensityAttr(50000.0)
#     sphereLight.AddTranslateOp().Set(Gf.Vec3f(3.0, 0, 1.5))

#     world.reset()
#     return world










def neutral_simulation():
    print("################ Starting Neutral Simulation ################")
    for _ in range(100000):
        # print("################################################## Step N°" + str(_))
        world.step(render=True)






if __name__ == "__main__":

    world = initialize_world()

    dy = Dynam2XLSim(world)


    neutral_simulation()
    simulation_app.close()

