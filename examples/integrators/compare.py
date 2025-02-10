from pydrake.all import *

import time
import matplotlib.pyplot as plt
import numpy as np
from dataclasses import dataclass

##
#
# Compare different integration schemes on a few toy examples.
#
##


@dataclass
class SimulationExample:
    """A little container for setting up different examples."""
    name: str
    xml: str
    use_hydroelastic: bool
    initial_state: np.array
    sim_time: float


def gripper():
    """A fake "gripper", where an object is wedged between two fixed joints."""
    name = "Gripper"
    xml = """
    <?xml version="1.0"?>
    <mujoco model="robot">
      <worldbody>
        <geom name="table" type="box" size="0.5 0.5 0.02" rgba="0.5 0.5 0.5 0.5"/>
        <body>
          <joint type="slide" />
          <geom name="post" type="box" pos="0 0 0.3" size="0.02 0.02 0.1" rgba="0.5 0.5 0.5 0.5"/>
          <geom name="finger1" type="box" pos="0.01 0.025 0.39" size="0.005 0.04 0.005" rgba="0.5 0.5 0.5 0.5"/>
          <geom name="finger2" type="box" pos="-0.01 0.025 0.39" size="0.005 0.04 0.005" rgba="0.5 0.5 0.5 0.5"/>
        </body>
        <body>
          <joint type="free" />
          <geom name="manipuland" type="box" pos="0.0 0.05 0.36" euler="10 0 0" size="0.00501 0.005 0.04" rgba="0.9 0.8 0.7 1.0"/>
        </body>
      </worldbody>
    </mujoco>
    """
    use_hydroelastic = True
    initial_state = np.array([0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    sim_time = 2.0
    return SimulationExample(name, xml, use_hydroelastic, initial_state, sim_time)


def ball_on_table():
    """A sphere is dropped on a table with some initial horizontal velocity."""
    name = "Ball on table"
    xml = """
    <?xml version="1.0"?>
    <mujoco model="robot">
    <worldbody>
        <geom name="table_top" type="box" pos="0.0 0.0 0.0" size="0.55 1.1 0.05" rgba="0.9 0.8 0.7 1"/>
        <body>
            <joint type="free"/>
            <geom name="object" type="sphere" pos="0.0 0.0 0.5" euler="80 0 0" size="0.1" rgba="1.0 1.0 1.0 1.0"/>
        </body>
    </worldbody>
    </mujoco>
    """
    use_hydroelastic = False
    initial_state = np.array(
        [1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0.])
    sim_time = 1.0
    return SimulationExample(name, xml, use_hydroelastic, initial_state, sim_time)


def double_pendulum():
    """A simple double pendulum (no contact)."""
    name = "Double Pendulum"
    xml = """
    <?xml version="1.0"?>
    <mujoco model="robot">
    <worldbody>
        <body>
        <joint type="hinge" axis="0 1 0" pos="0 0 0.1" damping="1e-3"/>
        <geom type="capsule" size="0.01 0.1"/>
        <body>
            <joint type="hinge" axis="0 1 0" pos="0 0 -0.1" damping="1e-3"/>
            <geom type="capsule" size="0.01 0.1" pos="0 0 -0.2"/>
        </body>
        </body>
    </worldbody>
    </mujoco>
    """
    use_hydroelastic = False
    initial_state = np.array([3.0, 0.1, 0.0, 0.0])
    sim_time = 2.0
    return SimulationExample(name, xml, use_hydroelastic, initial_state, sim_time)


def cylinder_hydro():
    """A cylinder with hydroelastic contact is dropped on the table."""
    name = "Cylinder w/ hydroelastic"
    xml = """
    <?xml version="1.0"?>
    <mujoco model="robot">
    <worldbody>
        <geom name="table_top" type="box" pos="0.0 0.0 0.0" size="0.55 1.1 0.05" rgba="0.9 0.8 0.7 1"/>
        <body>
            <joint type="free"/>
            <geom name="object" type="cylinder" pos="0.0 0.0 0.5" euler="80 0 0" size="0.1 0.1" rgba="1.0 1.0 1.0 1.0"/>
        </body>
    </worldbody>
    </mujoco>
    """
    use_hydroelastic = True
    initial_state = np.array(
        [1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0.])
    sim_time = 1.0
    return SimulationExample(name, xml, use_hydroelastic, initial_state, sim_time)


def cylinder_point():
    """A cylinder with point contact is dropped on the table."""
    name = "Cylinder w/ point contact"
    xml = """
    <?xml version="1.0"?>
    <mujoco model="robot">
    <worldbody>
        <geom name="table_top" type="box" pos="0.0 0.0 0.0" size="0.55 1.1 0.05" rgba="0.9 0.8 0.7 1"/>
        <body>
            <joint type="free"/>
            <geom name="object" type="cylinder" pos="0.0 0.0 0.5" euler="80 0 0" size="0.1 0.1" rgba="1.0 1.0 1.0 1.0"/>
        </body>
    </worldbody>
    </mujoco>
    """
    use_hydroelastic = False
    initial_state = np.array(
        [1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0.])
    sim_time = 1.0
    return SimulationExample(name, xml, use_hydroelastic, initial_state, sim_time)


def clutter():
    """Several spheres fall into a box."""
    name = "Clutter"
    xml = """
    <?xml version="1.0"?>
    <mujoco model="robot">
    <worldbody>
        <geom name="base" type="box" pos="0.0 0.0 0.0" size="0.05 0.05 0.002" rgba="0.5 0.5 0.5 0.3"/>
        <geom name="left" type="box" pos="0.05 0.0 0.05" size="0.002 0.05 0.05" rgba="0.5 0.5 0.5 0.3"/>
        <geom name="right" type="box" pos="-0.05 0.0 0.05" size="0.002 0.05 0.05" rgba="0.5 0.5 0.5 0.3"/>
        <geom name="front" type="box" pos="0.0 0.05 0.05" size="0.05 0.002 0.05" rgba="0.5 0.5 0.5 0.3"/>
        <geom name="back" type="box" pos="0.0 -0.05 0.05" size="0.05 0.002 0.05" rgba="0.5 0.5 0.5 0.3"/>
        <body>
            <joint type="free"/>
            <geom name="ball1" type="sphere" pos="0.0 0.0 0.05" size="0.01" rgba="1.0 1.0 1.0 1.0"/>
        </body>
        <body>
            <joint type="free"/>
            <geom name="ball2" type="sphere" pos="0.0001 0.0 0.07" size="0.01" rgba="1.0 1.0 1.0 1.0"/>
        </body>
        <body>
            <joint type="free"/>
            <geom name="ball3" type="sphere" pos="0.0 0.0001 0.09" size="0.01" rgba="1.0 1.0 1.0 1.0"/>
        </body>
        <body>
            <joint type="free"/>
            <geom name="ball4" type="sphere" pos="-0.0001 0.0 0.11" size="0.01" rgba="1.0 1.0 1.0 1.0"/>
        </body>
        <body>
            <joint type="free"/>
            <geom name="ball5" type="sphere" pos="0.0 -0.0001 0.13" size="0.01" rgba="1.0 1.0 1.0 1.0"/>
        </body>
    </worldbody>
    </mujoco>
    """
    use_hydroelastic = False
    initial_state = np.array(
        [1., 0., 0., 0., 0., 0., 0.,
         1., 0., 0., 0., 0., 0., 0.,
         1., 0., 0., 0., 0., 0., 0.,
         1., 0., 0., 0., 0., 0., 0.,
         1., 0., 0., 0., 0., 0., 0.,
         0., 0., 0., 0., 0., 0.,
         0., 0., 0., 0., 0., 0.,
         0., 0., 0., 0., 0., 0.,
         0., 0., 0., 0., 0., 0.,
         0., 0., 0., 0., 0., 0.])
    sim_time = 3.0
    return SimulationExample(name, xml, use_hydroelastic, initial_state, sim_time)


def create_scene(
    xml: str,
    time_step: float,
    hydroelastic: bool = False,
    meshcat: Meshcat = None,
):
    """
    Set up a drake system dyagram

    Args:
        xml: mjcf robot description
        time_step: dt for MultibodyPlant
        hydroelastic: whether to use hydroelastic contact
        meshcat: meshcat instance for visualization. Defaults to no visualization.

    Returns:
        The system diagram, the MbP within that diagram, and the logger instance
        used to keep track of time steps
    """
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder, time_step=time_step)

    parser = Parser(plant)
    parser.AddModelsFromString(xml, "xml")
    if time_step > 0:
        plant.set_discrete_contact_approximation(
            DiscreteContactApproximation.kLagged)
    plant.Finalize()

    if hydroelastic:
        sg_config = SceneGraphConfig()
        sg_config.default_proximity_properties.compliance_type = "compliant"
        scene_graph.set_config(sg_config)

    if meshcat is not None:
        AddDefaultVisualization(builder=builder, meshcat=meshcat)

    logger = LogVectorOutput(
        plant.get_state_output_port(),
        builder,
        publish_triggers={TriggerType.kForced},
        publish_period=0)
    logger.set_name("logger")

    diagram = builder.Build()
    return diagram, plant, logger


def run_simulation(
    example: SimulationExample,
    integrator: str,
    accuracy: float,
    max_step_size: float,
    visualize: bool = False
):
    """
    Run a short simulation, and report the time-steps used throughout.

    Args:
        example: container defining the scenario to simulate
        integrator: which integration strategy to use ("implicit_euler", "runge_kutta3", "convex", "discrete")
        accuracy: the desired accuracy (ignored for "discrete")
        max_step_size: the maximum (and initial) timestep dt
        visualize: flag for playing the sim in meshcat. Note that this breaks
                   timestep visualizations

    Returns:
        Timesteps (dt) throughout the simulation.
    """
    xml = example.xml
    use_hydroelastic = example.use_hydroelastic
    initial_state = example.initial_state
    sim_time = example.sim_time

    if visualize:
        meshcat = StartMeshcat()
    else:
        meshcat = None

    # We can use a more standard simulation setup and rely on a logger to
    # tell use the time step information. Note that in this case enabling
    # visualization messes with the time step report though.

    # Configure Drake's built-in error controlled integration
    config = SimulatorConfig()
    if integrator != "discrete":
        config.integration_scheme = integrator
    config.max_step_size = max_step_size
    config.accuracy = accuracy
    config.target_realtime_rate = 0.0
    config.use_error_control = True
    config.publish_every_time_step = True

    # Set up the system diagram and initial condition
    if integrator == "discrete":
        time_step = max_step_size
    else:
        time_step = 0.0
    diagram, plant, logger = create_scene(
        xml, time_step, use_hydroelastic, meshcat)
    context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, context)

    plant.SetPositionsAndVelocities(plant_context, initial_state)

    simulator = Simulator(diagram, context)
    ApplySimulatorConfig(config, simulator)
    simulator.Initialize()
    input("Waiting for meshcat... [ENTER] to continue")

    # Simulate
    if meshcat is not None:
        meshcat.StartRecording()
    start_time = time.time()
    simulator.AdvanceTo(sim_time)
    wall_time = time.time() - start_time
    if meshcat is not None:
        meshcat.StopRecording()
        meshcat.PublishRecording()

    print(f"\nWall clock time: {wall_time}\n")
    PrintSimulatorStatistics(simulator)

    # Get timesteps from the logger
    log = logger.FindLog(context)
    times = log.sample_times()
    timesteps = times[1:] - times[0:-1]

    # Keep meshcat instance alive for playback
    return np.asarray(timesteps), meshcat


if __name__ == "__main__":
    example = gripper()
    integrator = "convex"
    accuracy = 0.1

    time_steps, meshcat = run_simulation(
        example,
        integrator,
        accuracy,
        max_step_size=0.1,
        visualize=True,
    )

    # Plot stuff
    times = np.cumsum(time_steps)
    plt.title(
        f"{example.name} | {integrator} integrator | accuracy = {accuracy}")
    plt.plot(times, time_steps, "o")
    plt.ylim(1e-10, 1e0)
    plt.yscale("log")
    plt.xlabel("time (s)")
    plt.ylabel("step size (s)")
    plt.show()
