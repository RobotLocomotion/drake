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

        # Get the position of the suction cup C in the world frame
        X_WC = body_poses[self.body_idx]
        p_WC = X_WC.translation()

        # Compute distances from all geometries to the suction cup, up to a
        # maximum distance.
        max_dist = 0.4
        distances = query_object.ComputeSignedDistanceToPoint(p_WC, max_dist)

        # Compute a suction force on each geometry within the maximum distance
        spatial_forces = []
        for signed_distance in distances:
            # The body B that this geometry belongs to
            inspector = query_object.inspector()
            frame_id = inspector.GetFrameId(signed_distance.id_G)
            body = self.plant.GetBodyFromFrameId(frame_id)

            if body.index() != self.body_idx:  # Ignore the gripper body
                # The actual distance (scalar) to the suction cup
                distance = signed_distance.distance

                # The point P on the geometry closest to the suction cup,
                # expressed in the geometry frame
                p_GP = signed_distance.p_GN

                # Pose of the geometry frame in the body frame
                X_BG = inspector.GetPoseInFrame(signed_distance.id_G)

                # Position of point P, expressed in the body frame
                p_BP = X_BG @ p_GP

                # Position of point P, expressed in the world frame
                X_WB = body_poses[body.index()]
                p_WP = X_WB @ p_BP

                # Force on point P, expressed in the world frame
                magnitude = 10 * (max_dist - distance)
                assert magnitude >= 0.0
                direction = (p_WC - p_WP) / np.linalg.norm(p_WC - p_WP)
                f_WP_W = magnitude * direction

                # Assemble an associated spatial force
                spatial_force = ExternallyAppliedSpatialForce()
                spatial_force.body_index = body.index()
                spatial_force.p_BoBq_B = p_BP
                spatial_force.F_Bq_W = SpatialForce(np.zeros(3), f_WP_W)

                spatial_forces.append(spatial_force)

        output.set_value(spatial_forces)


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
q0_box = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.3])
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
