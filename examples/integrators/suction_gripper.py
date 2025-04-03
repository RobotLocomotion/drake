from pydrake.all import *
import numpy as np

##
#
# Simulate a simple suction gripper, modeled as a custom leaf system connected
# to MultibodyPlant's applied_spatial_force input port.
#
##

# System setup
meshcat = StartMeshcat()
builder = DiagramBuilder()

# Set up the plant model
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)
parser = Parser(plant)

# The gripper itself is just a sphere floating in space (for now)
radius = 0.02
color = np.array([0.8, 0.5, 0.5, 1.0])
pos = np.array([0.0, 0.0, 0.5])
gripper_body = plant.AddRigidBody(
    "gripper", SpatialInertia.SolidSphereWithDensity(1000, radius)
)
plant.RegisterVisualGeometry(
    gripper_body, RigidTransform(), Sphere(radius), "gripper", color
)
plant.RegisterCollisionGeometry(
    gripper_body,
    RigidTransform(),
    Sphere(radius),
    "gripper",
    CoulombFriction(1.0, 1.0),
)
plant.WeldFrames(
    plant.world_frame(), gripper_body.body_frame(), RigidTransform(pos)
)

# Manipuland(s)
box = parser.AddModelsFromUrl(
    "package://drake_models/ycb/003_cracker_box.sdf"
)[0]

# Ground with friction
plant.RegisterCollisionGeometry(
    plant.world_body(),
    RigidTransform([0.0, 0.0, -0.05]),
    Box(10, 10, 0.1),
    "ground",
    CoulombFriction(1.0, 1.0),
)

plant.Finalize()

# Connect to meshcat
AddDefaultVisualization(builder, meshcat)

# Build the diagram
diagram = builder.Build()
context = diagram.CreateDefaultContext()

# Set the initial state
q0_box = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1])
plant_context = plant.GetMyMutableContextFromRoot(context)
plant.SetPositions(plant_context, box, q0_box)

# Simulate
simulator = Simulator(diagram, context)
simulator.set_target_realtime_rate(1.0)
simulator.Initialize()

print("Waiting for meshcat... press [ENTER] to continue.")
input()

meshcat.StartRecording()
simulator.AdvanceTo(1.0)
meshcat.StopRecording()
meshcat.PublishRecording()

print("Waiting for meshcat... press [ENTER] to exit.")
input()
