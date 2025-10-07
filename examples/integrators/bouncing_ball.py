import numpy as np
import matplotlib.pyplot as plt

import time

from pydrake.all import (
    DiagramBuilder,
    RigidTransform,
    SpatialInertia,
    UnitInertia,
    CoulombFriction,
    AddMultibodyPlantSceneGraph,
    Simulator,
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


def create_bouncing_ball_sim(acc, dt, use_error_control):
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
    ground_height = -0.05 + 0.001962  # zero potential energy at rest
    ground_shape = Box(2.0, 2.0, 2.0)
    ground_pose = RigidTransform([0, 0, -1.0 + ground_height])
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

    #     double_pendulum_xml = """
    # <?xml version="1.0"?>
    # <mujoco model="double_pendulum">
    # <worldbody>
    #   <body>
    #   <joint type="hinge" axis="0 1 0" pos="0 0 0.1" damping="0.0"/>
    #   <geom type="capsule" size="0.01 0.1"/>
    #   <body>
    #     <joint type="hinge" axis="0 1 0" pos="0 0 -0.1" damping="0.0"/>
    #     <geom type="capsule" size="0.01 0.1" pos="0 0 -0.2"/>
    #   </body>
    #   </body>
    # </worldbody>
    # </mujoco>
    # """
    #     Parser(plant).AddModelsFromString(double_pendulum_xml, "xml")
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
    # plant.SetPositions(plant_context, [3.0, 1.0])

    simulator = Simulator(diagram, context)
    config = SimulatorConfig()
    config.integration_scheme = "convex"
    config.max_step_size = dt
    config.use_error_control = use_error_control
    config.accuracy = acc
    ApplySimulatorConfig(config, simulator)

    if config.integration_scheme == "convex":
        ci = simulator.get_mutable_integrator()
        ci.set_plant(plant)
        ci_params = ci.get_solver_parameters()
        ci.set_solver_parameters(ci_params)

    return simulator, plant, None


time_steps = [1e-2, 3e-3, 1e-3, 3e-4, 1e-4, 3e-5, 1e-5]
accuracies = [1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8]
wall_times = []
energy_errors = []

for time_step in time_steps:
    print(f"Running at dt={time_step}")
    simulator, plant, ball_body = create_bouncing_ball_sim(
        1e-1, time_step, False
    )

    # Simulate and collect data
    sim_time = 5.0
    simulator.Initialize()

    context = simulator.get_context()
    plant_context = plant.GetMyContextFromRoot(context)
    e0 = plant.CalcPotentialEnergy(plant_context) + plant.CalcKineticEnergy(
        plant_context
    )

    st = time.time()
    simulator.AdvanceTo(sim_time)
    wall_time = time.time() - st

    eF = plant.CalcPotentialEnergy(plant_context) + plant.CalcKineticEnergy(
        plant_context
    )

    # PrintSimulatorStatistics(simulator)

    print(f"==> Wall time: {wall_time} s")
    energy_loss = abs(eF - e0)
    print(f"==> Energy loss: {energy_loss} %")

    energy_errors.append(energy_loss)
    wall_times.append(wall_time)

print("time_steps =", repr(time_steps))
print("wall_times =", repr(wall_times))
print("energy_errors =", repr(energy_errors))

time_steps = np.array(time_steps)
plt.plot(time_steps, energy_errors, "o-")
plt.plot(time_steps, 1e4 * time_steps, "k--", label="O(dt)")
plt.plot(time_steps, 1e4 * time_steps**2, "k-.", label="O(dt^2)")
plt.legend()

plt.xscale("log")
plt.yscale("log")
plt.gca().invert_xaxis()
plt.xlabel("Time Step (s)")
plt.ylabel("Energy Error (J)")
plt.grid()

plt.show()


# # Plotting
# plt.figure(figsize=(8, 4))
# plt.subplot(2, 1, 1)
# plt.plot(time_steps, z_positions, label="Ball height (z)")
# plt.xlabel("Time [s]")
# plt.ylabel("Height [m]")
# plt.grid(True)
# plt.legend()

# plt.subplot(2, 1, 2)
# plt.plot(time_steps, energies, label="Total energy")
# plt.xlabel("Time [s]")
# plt.ylabel("Energy [J]")
# plt.grid(True)
# plt.legend()

# plt.tight_layout()
# plt.show()
