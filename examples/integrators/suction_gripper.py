import time
import argparse
from pydrake.all import *
import numpy as np
from pydrake.multibody.cenic import IcfSolverParameters

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
        self.plant = plant
        self.body_idx = body_idx
        self.force_radius = force_radius
        self.max_force = max_force

        # Input ports take body poses and geometry info
        self.body_poses_input_port = self.DeclareAbstractInputPort(
            "body_poses", AbstractValue.Make([RigidTransform()])
        )
        self.query_object_input_port = self.DeclareAbstractInputPort(
            "query_object", AbstractValue.Make(QueryObject())
        )

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

            # Skip parts of the gripper itself
            if body.name() != "gripper_base" and body.name()[:7] != "bellows":
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


def add_gripper_mbp_elements(plant, scene_graph):
    """Adds a model of a suction gripper to the given MultibodyPlant.

    This adds collision and visual geometries, as well as springs to model the
    bellows. Does not add the actual suction forces, which are provided by the
    SuctionGripper leaf system.
    """
    # Basic parameters
    base_length, base_width, base_height = 0.1, 0.08, 0.02
    bellows_radius = 0.01
    bellows_height = 0.015
    bellows_offset = -0.02
    bellows_stiffness = 1.0
    bellows_damping = 1.0

    # The gripper base is a simple box
    gripper_base = plant.AddRigidBody(
        "gripper_base",
        SpatialInertia.SolidBoxWithDensity(
            1000, base_length, base_width, base_height
        ),
    )
    plant.RegisterVisualGeometry(
        gripper_base,
        RigidTransform(),
        Box(base_length, base_width, base_height),
        "gripper_base",
        np.array([0.5, 0.5, 0.5, 1.0]),
    )
    gripper_base_geom_id = plant.RegisterCollisionGeometry(
        gripper_base,
        RigidTransform(),
        Box(base_length, base_width, base_height),
        "gripper_base",
        CoulombFriction(1.0, 1.0),
    )
    plant.WeldFrames(
        plant.world_frame(),
        gripper_base.body_frame(),
        RigidTransform([0.0, 0.0, 0.5]),
    )

    # Add little suction cup models across the gripper base
    xpos = np.linspace(-base_length / 2 + 0.02, base_length / 2 - 0.02, 3)
    ypos = np.linspace(-base_width / 2 + 0.02, base_width / 2 - 0.02, 2)

    collision_filter_set = GeometrySet(gripper_base_geom_id)
    pressure_sources = []
    i = 0
    for x in xpos:
        for y in ypos:
            pos = np.array([x, y, -base_height / 2])

            # Add a small body on the gripper base to model the pressure source
            pressure_source = plant.AddRigidBody(
                f"pressure_source_{i}",
                SpatialInertia.SolidSphereWithDensity(1000, 0.002),
            )
            plant.RegisterVisualGeometry(
                pressure_source,
                RigidTransform(),
                Sphere(0.002),
                f"pressure_source_{i}",
                np.array([0.8, 0.2, 0.2, 0.5]),
            )
            plant.WeldFrames(
                gripper_base.body_frame(),
                pressure_source.body_frame(),
                RigidTransform(pos),
            )
            pressure_sources.append(pressure_source)

            # The bellows are lightweight cylinders
            bellows = plant.AddRigidBody(
                f"bellows_{i}",
                SpatialInertia.SolidCylinderWithDensity(
                    10,
                    bellows_radius,
                    bellows_height,
                    np.array([0.0, 0.0, 1.0]),
                ),
            )
            plant.RegisterVisualGeometry(
                bellows,
                RigidTransform(),
                Cylinder(bellows_radius, bellows_height),
                f"bellows_{i}",
                np.array([0.1, 0.1, 0.1, 1.0]),
            )
            bellows_geom_id = plant.RegisterCollisionGeometry(
                bellows,
                RigidTransform(),
                Cylinder(bellows_radius, bellows_height),
                f"bellows_{i}",
                CoulombFriction(1.0, 1.0),
            )

            # The bellows are attached to the pressure source with a
            # spring-damper
            bellows_joint = plant.AddJoint(
                PrismaticJoint(
                    f"bellows_joint_{i}",
                    pressure_source.body_frame(),
                    bellows.body_frame(),
                    [0.0, 0.0, 1.0],
                    damping=bellows_damping,
                )
            )
            bellows_joint.set_default_translation(bellows_offset)
            plant.AddForceElement(
                PrismaticSpring(
                    bellows_joint,
                    nominal_position=bellows_offset,
                    stiffness=bellows_stiffness,
                )
            )

            # Add the bellows to the collision filter set
            collision_filter_set.Add(bellows_geom_id)

            i += 1

    # Enable collisions between the bellows and the gripper base, even though
    # these are directly connected by a joint.
    collision_filter = CollisionFilterDeclaration().AllowWithin(
        collision_filter_set
    )

    return pressure_sources, collision_filter


def run_simulation(
    mbp_time_step, 
    integrator, 
    accuracy,
    use_error_control,
    max_time_step,
    visualize
):
    """Run the simulation with the given parameters."""

    # System setup
    meshcat = StartMeshcat()
    builder = DiagramBuilder()

    # Set up the plant model
    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder, time_step=mbp_time_step
    )
    parser = Parser(plant)

    # Gripper geometries
    pressure_sources, gripper_collision_filter = add_gripper_mbp_elements(
        plant, scene_graph
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

    # Allow collisions between bellows and grippper base
    scene_graph.collision_filter_manager().Apply(gripper_collision_filter)

    # Connect a suction gripper model for each suction cup in the gripper
    force_multiplexer = builder.AddSystem(
        ExternallyAppliedSpatialForceMultiplexer(len(pressure_sources))
    )

    for i, suction_cup in enumerate(pressure_sources):
        suction_model = builder.AddSystem(
            SuctionGripper(
                plant,
                suction_cup.index(),
                force_radius=0.04,
                max_force=50.0,
            )
        )
        builder.Connect(
            suction_model.GetOutputPort("force_output"),
            force_multiplexer.get_input_port(i),
        )

        builder.Connect(
            plant.get_body_poses_output_port(),
            suction_model.body_poses_input_port,
        )
        builder.Connect(
            scene_graph.get_query_output_port(),
            suction_model.query_object_input_port,
        )

    builder.Connect(
        force_multiplexer.get_output_port(),
        plant.get_applied_spatial_force_input_port(),
    )

    # Use hydroelastic contact
    sg_config = SceneGraphConfig()
    sg_config.default_proximity_properties.compliance_type = "compliant"
    scene_graph.set_config(sg_config)

    # Connect to meshcat
    if visualize:
        AddDefaultVisualization(builder, meshcat)

    # Build the diagram
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()

    # Set the initial state
    q0_box = np.array([1.0, 0.0, 0.01, 0.1, 0.0, 0.0, 0.42])
    plant_context = plant.GetMyMutableContextFromRoot(context)
    plant.SetPositions(plant_context, box, q0_box)

    # Set up the simulator
    config = SimulatorConfig()
    if mbp_time_step == 0.0:
        config.integration_scheme = integrator
    config.accuracy = accuracy
    config.target_realtime_rate = 0.0
    config.use_error_control = use_error_control
    config.max_step_size = max_time_step

    simulator = Simulator(diagram, context)
    ApplySimulatorConfig(config, simulator)
    if config.integration_scheme == "cenic":
        ci = simulator.get_mutable_integrator()

        # Make sure we can set ICF parameters
        icf_params = IcfSolverParameters()
        icf_params.min_tolerance = 1e-7
        ci.SetSolverParameters(icf_params)
        assert ci.get_solver_parameters().min_tolerance == 1e-7

    meshcat.StartRecording()
    simulator.Initialize()

    if visualize:
        print("Waiting for meshcat... press [ENTER] to continue.")
        input()

    # Simulate
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
        default="cenic",
        help="The integrator to use. default: cenic",
    )
    parser.add_argument(
        "--accuracy",
        type=float,
        default=1e-3,
        help="The accuracy to use. default: 1e-3",
    )
    parser.add_argument(
        "--no_error_control",
        action="store_true",
        help="Disables error control.",
    )
    parser.add_argument(
        "--max_time_step",
        type=float,
        default=0.1,
        help="The maximum time step to use. default: 0.1",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="If specified, do not visualize in meshcat.",
    )
    args = parser.parse_args()

    # Run the simulation
    run_simulation(
        mbp_time_step=args.mbp_time_step,
        integrator=args.integrator,
        accuracy=args.accuracy,
        use_error_control=not args.no_error_control,
        max_time_step=args.max_time_step,
        visualize=not args.headless
    )
