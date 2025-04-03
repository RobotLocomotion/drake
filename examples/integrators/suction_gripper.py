import time
import argparse
from pydrake.all import *
import numpy as np

##
#
# Simulate a simple suction gripper, modeled as a custom leaf system connected
# to MultibodyPlant's applied_spatial_force input port.
#
##

class SuctionGripper(LeafSystem):
    """A super simple suction gripper model.

    This model pulls all geometries within a certain distance towards the
    gripper. The magnitude of the pulling force interpolates linearly from zero
    at the boundary to a user-defined maximum value.
    """

    def __init__(self, plant, body_idx, force_radius, max_force):
        """Construct the suction gripper model.

        Args:
            plant: A model of the MultibodyPlant which the gripper acts upon.
            body_idx: The index of the body to which the gripper is attached.
            force_radius: The maximum distance from the gripper at which the
                suction force is applied.
            max_force: The maximum suction force.
        """
        super().__init__()
        self.set_name("suction_gripper")
        self.plant = plant
        self.body_idx = body_idx
        self.force_radius = force_radius
        self.max_force = max_force

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

        # Compute distances from all geometries that are within
        # self.force_radius of the suction cup
        distances = query_object.ComputeSignedDistanceToPoint(
            p_WC, self.force_radius
        )

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
                scale = self.max_force / self.force_radius
                magnitude = scale * (self.force_radius - distance)
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

def run_simulation(mbp_time_step, integrator, accuracy, use_error_control):
    """Run the simulation with the given parameters."""

    # System setup
    meshcat = StartMeshcat()
    builder = DiagramBuilder()

    # Set up the plant model
    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder, time_step=mbp_time_step
    )
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
    suction_model = builder.AddSystem(
        SuctionGripper(
            plant,
            gripper_body.index(),
            force_radius=0.2,
            max_force=5.0,
        )
    )

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
    q0_box = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.43])
    plant_context = plant.GetMyMutableContextFromRoot(context)
    plant.SetPositions(plant_context, box, q0_box)

    # Set up the simulator
    config = SimulatorConfig()
    if mbp_time_step == 0.0:
        config.integration_scheme = integrator
    config.accuracy = accuracy
    config.target_realtime_rate = 0.0
    config.use_error_control = use_error_control

    simulator = Simulator(diagram, context)
    ApplySimulatorConfig(config, simulator)
    simulator.Initialize()

    print("Waiting for meshcat... press [ENTER] to continue.")
    input()

    # Simulate
    meshcat.StartRecording()
    st = time.time()
    simulator.AdvanceTo(1.0)
    wall_time = time.time() - st
    meshcat.StopRecording()
    meshcat.PublishRecording()

    print(f"\nWall time: {wall_time}\n")

    # Print a summary of solver statistics
    PrintSimulatorStatistics(simulator)

if __name__ == "__main__":
    # Get command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mbp_time_step",
        type=float,
        default=0.0,
        help="The MultibodyPlant time step. default: 0.0",
    )
    parser.add_argument(
        "--integrator",
        type=str,
        default="convex",
        help="The integrator to use. default: convex",
    )
    parser.add_argument(
        "--accuracy",
        type=float,
        default=0.1,
        help="The accuracy to use. default: 0.1",
    )
    parser.add_argument(
        "--no_error_control",
        action="store_true",
        help="Disables error control.",
    )
    args = parser.parse_args()

    # Run the simulation
    run_simulation(
        mbp_time_step=args.mbp_time_step,
        integrator=args.integrator,
        accuracy=args.accuracy,
        use_error_control=not args.no_error_control,
    )
