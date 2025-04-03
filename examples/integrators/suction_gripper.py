from pydrake.all import *
import numpy as np

##
#
# Simulate a simple suction gripper, modeled as a custom leaf system connected
# to MultibodyPlant's applied_spatial_force input port.
#
##

class SuctionGripper(LeafSystem):
    """A simple suction gripper model."""

    def __init__(self, plant, body_idx):
        """Construct the suction gripper model.

        Args:
            plant: A model of the MultibodyPlant which the gripper acts upon.
            body_idx: The index of the body to which the gripper is attached.
        """
        super().__init__()
        self.set_name("suction_gripper")
        self.plant = plant
        self.body_idx = body_idx

        # Input ports take body poses and geometry info
        self.body_poses_input_port = self.DeclareAbstractInputPort(
            "body_poses", AbstractValue.Make([RigidTransform()]))
        self.query_object_input_port = self.DeclareAbstractInputPort(
            "query_object", AbstractValue.Make(QueryObject()))

        # Output port sends externally applied spatial forces
        self.DeclareAbstractOutputPort(
            "force_output",
            lambda: AbstractValue.Make([ExternallyAppliedSpatialForce()]),
            self.CalcSpatialForces,
        )

    def CalcSpatialForces(self, context, output):
        """Compute the spatial forces applied by the gripper."""
        body_poses = self.body_poses_input_port.Eval(context)
        query_object = self.query_object_input_port.Eval(context)

        my_force = ExternallyAppliedSpatialForce()
        my_force.body_index = BodyIndex(2)
        my_force.p_BoBq_B = np.array([0.0, 0.0, 0.0])
        my_force.F_Bq_W = SpatialForce(
            np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 10.0])
        )

        output.set_value([my_force])


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

# Connect the suction gripper model
suction_model = builder.AddSystem(SuctionGripper(plant, gripper_body.index()))

builder.Connect(
    suction_model.GetOutputPort("force_output"),
    plant.get_applied_spatial_force_input_port(),
)
builder.Connect(
    plant.get_body_poses_output_port(),
    suction_model.body_poses_input_port,
)
builder.Connect(
    scene_graph.get_query_output_port(),
    suction_model.query_object_input_port,
)


# Connect to meshcat
AddDefaultVisualization(builder, meshcat)

# Build the diagram
diagram = builder.Build()
context = diagram.CreateDefaultContext()

# Set the initial state
q0_box = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.1])
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

# print("Waiting for meshcat... press [ENTER] to exit.")
# input()
