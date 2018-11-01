import matplotlib.pyplot as plt

from pydrake.common import FindResourceOrThrow
from pydrake.geometry import (ConnectDrakeVisualizer, SceneGraph)
from pydrake.lcm import DrakeLcm
from pydrake.multibody.multibody_tree import UniformGravityFieldElement
from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant
from pydrake.multibody.multibody_tree.parsing import AddModelFromSdfFile
from pydrake.systems.drawing import plot_system_graphviz
from pydrake.systems.framework import DiagramBuilder

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--test",
                    action='store_true',
                    help="Causes the script to run without blocking for "
                         "user input.",
                    default=False)
args = parser.parse_args()

file_name = FindResourceOrThrow(
    "drake/examples/multibody/cart_pole/cart_pole.sdf")
builder = DiagramBuilder()
scene_graph = builder.AddSystem(SceneGraph())
cart_pole = builder.AddSystem(MultibodyPlant())
cart_pole.RegisterAsSourceForSceneGraph(scene_graph)
AddModelFromSdfFile(file_name=file_name, plant=cart_pole)
cart_pole.AddForceElement(UniformGravityFieldElement([0, 0, -9.81]))
cart_pole.Finalize()
assert cart_pole.geometry_source_is_registered()

builder.Connect(
    scene_graph.get_query_output_port(),
    cart_pole.get_geometry_query_input_port())
builder.Connect(
    cart_pole.get_geometry_poses_output_port(),
    scene_graph.get_source_pose_port(cart_pole.get_source_id()))
builder.ExportInput(cart_pole.get_actuation_input_port())

ConnectDrakeVisualizer(builder=builder, scene_graph=scene_graph)

diagram = builder.Build()
diagram.set_name("graphviz_example")

plot_system_graphviz(diagram, max_depth=2)

if not args.test:
    plt.show()
