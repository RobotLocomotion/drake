"""Provides an example translation of `cart_pole_passive_simluation.cc`."""

import argparse

from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.visualization import AddDefaultVisualization


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--target_realtime_rate", type=float, default=1.0,
        help="Desired rate relative to real time.  See documentation for "
             "Simulator::set_target_realtime_rate() for details.")
    parser.add_argument(
        "--simulation_time", type=float, default=10.0,
        help="Desired duration of the simulation in seconds.")
    parser.add_argument(
        "--time_step", type=float, default=0.,
        help="If greater than zero, the plant is modeled as a system with "
             "discrete updates and period equal to this time_step. "
             "If 0, the plant is modeled as a continuous system.")
    args = parser.parse_args()

    builder = DiagramBuilder()
    cart_pole, scene_graph = AddMultibodyPlantSceneGraph(
        builder=builder, time_step=args.time_step)
    Parser(plant=cart_pole).AddModelsFromUrl(
        url="package://drake/examples/multibody/cart_pole/cart_pole.sdf")
    cart_pole.Finalize()

    AddDefaultVisualization(builder=builder)
    diagram = builder.Build()

    diagram_context = diagram.CreateDefaultContext()
    cart_pole_context = cart_pole.GetMyMutableContextFromRoot(diagram_context)

    cart_pole.get_actuation_input_port().FixValue(cart_pole_context, 0)

    cart_slider = cart_pole.GetJointByName("CartSlider")
    pole_pin = cart_pole.GetJointByName("PolePin")
    cart_slider.set_translation(context=cart_pole_context, translation=0.)
    pole_pin.set_angle(context=cart_pole_context, angle=2.)

    simulator = Simulator(diagram, diagram_context)
    simulator.set_publish_every_time_step(False)
    simulator.set_target_realtime_rate(args.target_realtime_rate)
    simulator.Initialize()
    simulator.AdvanceTo(args.simulation_time)


if __name__ == "__main__":
    main()
