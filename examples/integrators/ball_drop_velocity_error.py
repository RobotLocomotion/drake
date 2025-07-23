##
#
# Simulate a ball falling on a table with a few different choices of time step,
# and make plots of the resulting position and velocity error.
#
##

import matplotlib.pyplot as plt
from pydrake.all import *

plt.rcParams["font.family"] = "serif"
plt.rcParams["font.size"] = 12


def run_simulation(time_step):
    """Run a simulation of a ball falling on a table. Return a trajectory with
    the ball's height and vertical velocity.
    """
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    Parser(plant).AddModels(
        url="package://drake/examples/integrators/ball_on_table.xml"
    )
    plant.Finalize()
    logger = LogVectorOutput(
        plant.get_state_output_port(),
        builder,
        publish_triggers={TriggerType.kForced},
        publish_period=0,
    )
    diagram = builder.Build()

    context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, context)
    plant.SetPositionsAndVelocities(
        plant_context,
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    )

    simulator = Simulator(diagram, context)
    config = SimulatorConfig()
    config.integration_scheme = "convex"
    config.use_error_control = False
    config.max_step_size = time_step
    config.publish_every_time_step = True
    config.target_realtime_rate = 0.0
    ApplySimulatorConfig(config, simulator)
    simulator.get_mutable_integrator().set_plant(plant)

    simulator.Initialize()
    simulator.AdvanceTo(1.0)
    PrintSimulatorStatistics(simulator)

    log = logger.FindLog(context)
    times = log.sample_times()
    state = log.data()

    q = state[6:7, :]
    v = state[-1:, :]
    q_traj = PiecewisePolynomial.ZeroOrderHold(times, q)
    v_traj = PiecewisePolynomial.ZeroOrderHold(times, v)

    return q_traj, v_traj


if __name__ == "__main__":
    # Run simulation with several time steps
    qs_1, vs_1 = run_simulation(0.001)
    qs_2, vs_2 = run_simulation(0.005)

    # Compare the solutions
    times = np.arange(0.22, 0.32, 0.005)
    qs_1 = [qs_1.value(t)[0] for t in times]
    qs_2 = [qs_2.value(t)[0] for t in times]
    vs_1 = [vs_1.value(t)[0] for t in times]
    vs_2 = [vs_2.value(t)[0] for t in times]
    q_err = np.abs(np.array(qs_1) - np.array(qs_2))
    v_err = np.abs(np.array(vs_1) - np.array(vs_2))

    fig, ax = plt.subplots(2, 2, figsize=(6, 4), sharex=True)

    ax[0, 0].plot(times, qs_1, "k-")
    ax[0, 0].plot(times, qs_2, "k:")
    ax[0, 0].set_ylabel("Position")

    ax[0, 1].plot(times, vs_1, "k-")
    ax[0, 1].plot(times, vs_2, "k:")
    ax[0, 1].set_ylabel("Velocity")

    ax[1, 0].plot(times, q_err, "k-")
    ax[1, 0].set_ylabel("Position Error")
    ax[1, 0].set_ylim(-0.01, 0.2)

    ax[1, 1].plot(times, v_err, "k-")
    ax[1, 1].set_ylabel("Velocity Error")
    ax[1, 1].set_ylim(-0.1, 2.5)

    fig.supxlabel("Time (s)", fontsize=12)

    plt.tight_layout()
    plt.show()
