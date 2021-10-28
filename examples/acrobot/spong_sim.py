"""A main() program (plus an reusable standalone function) that simulates a
spong-controlled acrobot.
"""

import argparse
import sys

from pydrake.examples.acrobot import (
    AcrobotPlant,
    AcrobotSpongController,
    AcrobotState,
)
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import LogVectorOutput

from drake.examples.acrobot.acrobot_io import load_scenario, save_output


def simulate(*, initial_state, controller_params, t_final, tape_period):
    """Simulates an Acrobot + Spong controller from the given initial state and
    parameters until the given final time.  Returns the state sampled at the
    given tape_period.
    """
    builder = DiagramBuilder()
    plant = builder.AddSystem(AcrobotPlant())
    controller = builder.AddSystem(AcrobotSpongController())

    builder.Connect(plant.get_output_port(0), controller.get_input_port(0))
    builder.Connect(controller.get_output_port(0), plant.get_input_port(0))
    state_logger = LogVectorOutput(plant.get_output_port(0), builder,
                                   tape_period)

    diagram = builder.Build()
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    plant_context = diagram.GetMutableSubsystemContext(plant, context)
    controller_context = diagram.GetMutableSubsystemContext(
        controller, context)

    plant_context.SetContinuousState(initial_state)
    controller_context.get_mutable_numeric_parameter(0).SetFromVector(
        controller_params)

    simulator.AdvanceTo(t_final)

    x_tape = state_logger.FindLog(context).data()
    return x_tape


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario", metavar="*.yaml", required=True,
        help="Scenario file to load (required).")
    parser.add_argument(
        "--output", metavar="*.yaml", required=True,
        help="Output file to save (required).")
    args = parser.parse_args()
    scenario = load_scenario(filename=args.scenario)
    x_tape = simulate(**scenario)
    text = save_output(x_tape=x_tape)
    with open(args.output, 'w') as handle:
        handle.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
