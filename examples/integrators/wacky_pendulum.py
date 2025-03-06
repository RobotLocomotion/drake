import argparse
from pydrake.all import *
import numpy as np

##
#
# Simulate a simple pendulum with a strange controller that applies torques
# 
#   u = - sin(2πv).
#
# This gives a controller with various unstable regions, and stable equilibria
# around v = 0, v = 1, v = 2, ...
#
##

class WackyController(LeafSystem):
    def __init__(self):
        super().__init__()
        self.state_input_port = self.DeclareVectorInputPort(name="state", size=2)
        self.DeclareVectorOutputPort(name="control", size=1, calc=self.CalcOutput)

    def CalcOutput(self, context, output):
        x = self.state_input_port.Eval(context)
        u = - np.sin(2 * np.pi * x[1])
        print(f"v = {x[1]}")
        output.SetFromVector([u])

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--v0",
        type=float,
        default=0.1,
        help="Initial velocity",
    )
    parser.add_argument(
        "--integrator",
        type=str,
        default="convex",
        help="Integrator to use, e.g., 'convex', 'implicit_euler'."
    )
    parser.add_argument(
        "--accuracy",
        type=float,
        default=0.1,
        help="Integrator accuracy. Default: 0.1.",
    )
    parser.add_argument(
        "--sim_time",
        type=float,
        default=10.0,
        help="Simulation time (in seconds)."
    )
    args = parser.parse_args()

    # Set up system diagram
    meshcat = StartMeshcat()
    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    Parser(plant).AddModels(url="package://drake/examples/pendulum/Pendulum.urdf")
    plant.mutable_gravity_field().set_gravity_vector([0.0, 0.0, 0.0])
    plant.Finalize()

    ctrl = builder.AddSystem(WackyController())
    builder.Connect(
        ctrl.get_output_port(),
        plant.get_actuation_input_port()
    )
    builder.Connect(
        plant.get_state_output_port(),
        ctrl.get_input_port()
    )

    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    diagram = builder.Build()

    # Set the initial state
    context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, context)
    plant.SetVelocities(plant_context, [args.v0])

    # Set up the simulator
    config = SimulatorConfig()
    config.integration_scheme = args.integrator
    config.accuracy = args.accuracy
    config.target_realtime_rate = 0.0
    config.use_error_control = True
    config.publish_every_time_step = True

    simulator = Simulator(diagram, context)
    ApplySimulatorConfig(config, simulator)
    simulator.Initialize()

    print(f"Running with {args.integrator} integration at accuracy = {args.accuracy}.")
    input("Waiting for meshcat... press [ENTER] to continue")

    # Run the sim
    meshcat.StartRecording()
    simulator.AdvanceTo(args.sim_time)
    meshcat.StopRecording()
    meshcat.PublishRecording()

    # Print a summary of solver statistics
    PrintSimulatorStatistics(simulator)
    