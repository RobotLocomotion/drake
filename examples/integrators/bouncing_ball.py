import numpy as np
import matplotlib.pyplot as plt

from pydrake.all import (
    MultibodyPlant,
    DiagramBuilder,
    RigidTransform,
    SpatialInertia,
    UnitInertia,
    CoulombFriction,
    AddMultibodyPlantSceneGraph,
    Simulator,
    PrintSimulatorStatistics,
    SimulatorConfig,
    ApplySimulatorConfig,
)
from pydrake.geometry import (
    Box,
    Sphere,
    ProximityProperties,
    AddContactMaterial,
)
from pydrake.multibody.math import SpatialVelocity

##
#
# Bouncing ball demo to test energy conservation. Adapted from
# https://stackoverflow.com/questions/79728775
#
##


def create_bouncing_ball_sim():
    builder = DiagramBuilder()

    # Discrete plant with small time step
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)

    # Proximity properties for bouncy contact
    props = ProximityProperties()
    AddContactMaterial(
        dissipation=0.0,
        point_stiffness=1e3,
        friction=CoulombFriction(0.8, 0.6),
        properties=props,
    )

    # Ground
    ground_shape = Box(2.0, 2.0, 0.1)
    ground_pose = RigidTransform([0, 0, -0.05])
    plant.RegisterCollisionGeometry(
        plant.world_body(),
        ground_pose,
        ground_shape,
        "ground_collision",
        props,
    )
    plant.RegisterVisualGeometry(
        plant.world_body(),
        ground_pose,
        ground_shape,
        "ground_visual",
        [0.5, 0.5, 0.5, 1.0],
    )

    # Ball
    radius = 0.05
    mass = 0.1
    inertia = UnitInertia.SolidSphere(radius)
    spatial_inertia = SpatialInertia(
        mass=mass, p_PScm_E=np.zeros(3), G_SP_E=inertia
    )

    ball_body = plant.AddRigidBody("ball", spatial_inertia)
    plant.RegisterCollisionGeometry(
        ball_body, RigidTransform(), Sphere(radius), "ball_collision", props
    )
    plant.RegisterVisualGeometry(
        ball_body,
        RigidTransform(),
        Sphere(radius),
        "ball_visual",
        [0.8, 0.1, 0.1, 1.0],
    )

    # Gravity
    plant.mutable_gravity_field().set_gravity_vector([0, 0, -9.81])
    plant.Finalize()

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()

    plant_context = plant.GetMyMutableContextFromRoot(context)
    plant.SetFreeBodyPose(
        plant_context, ball_body, RigidTransform([0, 0, 1.0])
    )
    plant.SetFreeBodySpatialVelocity(
        ball_body, SpatialVelocity(np.zeros(3), np.zeros(3)), plant_context
    )

    simulator = Simulator(diagram, context)
    config = SimulatorConfig()
    config.integration_scheme = "convex"
    config.max_step_size = 0.1
    config.use_error_control = True
    config.accuracy = 1e-6
    ApplySimulatorConfig(config, simulator)

    ci = simulator.get_mutable_integrator()
    ci.set_plant(plant)
    ci_params = ci.get_solver_parameters()
    ci_params.error_estimation_strategy = "richardson"
    ci.set_solver_parameters(ci_params)

    return simulator, plant, ball_body


# Run simulation
simulator, plant, ball_body = create_bouncing_ball_sim()

# Simulate and collect data
time_steps = []
z_positions = []
energies = []

sim_time = 10.0
dt = 0.01
simulator.Initialize()

while simulator.get_context().get_time() < sim_time:
    context = simulator.get_context()
    plant_context = plant.GetMyContextFromRoot(context)
    pose = plant.EvalBodyPoseInWorld(plant_context, ball_body)
    z = pose.translation()[2]
    t = context.get_time()

    e = plant.CalcPotentialEnergy(plant_context) + plant.CalcKineticEnergy(
        plant_context
    )

    z_positions.append(z)
    energies.append(e)
    time_steps.append(t)

    simulator.AdvanceTo(t + dt)

PrintSimulatorStatistics(simulator)

# Plotting
plt.figure(figsize=(8, 4))
plt.subplot(2, 1, 1)
plt.plot(time_steps, z_positions, label="Ball height (z)")
plt.xlabel("Time [s]")
plt.ylabel("Height [m]")
plt.grid(True)
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(time_steps, energies, label="Total energy")
plt.xlabel("Time [s]")
plt.ylabel("Energy [J]")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
