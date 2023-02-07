"""Shows a quick demo of pydrake.systems.drawing.

To see this interactively:
  bazel run //bindings/pydrake/systems:drawing_graphviz_example
"""

# We must render the figures somehow.
# The Agg backend (creating a PNG image) is suitably cross-platform.
# Users should feel free to use a different back-end in their own code.
import os
os.environ['MPLBACKEND'] = 'Agg'  # noqa

# Now that the environment is set up, it's safe to import matplotlib, etc.
import matplotlib.pyplot as plt
import webbrowser

from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.drawing import plot_graphviz, plot_system_graphviz
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization

# If running under `bazel run`, output to cwd so the user can find it.
# If running under `bazel test` avoid polluting the test's cwd.
for env_name in ['BUILD_WORKING_DIRECTORY', 'TEST_TMPDIR']:
    if env_name in os.environ:
        os.chdir(os.environ[env_name])

builder = DiagramBuilder()
cart_pole, scene_graph = AddMultibodyPlantSceneGraph(
    builder=builder, time_step=0.0)
Parser(plant=cart_pole).AddModelsFromUrl(
    url="package://drake/examples/multibody/cart_pole/cart_pole.sdf")

plt.figure(figsize=(11, 8.5), dpi=300)
plot_graphviz(cart_pole.GetTopologyGraphvizString())
plt.savefig('cart_pole_topology.png')
assert os.path.exists('cart_pole_topology.png')

cart_pole.Finalize()
builder.ExportInput(cart_pole.get_actuation_input_port())

AddDefaultVisualization(builder=builder)

diagram = builder.Build()
diagram.set_name("graphviz_example")

plt.figure(figsize=(11, 8.5), dpi=300)
plot_system_graphviz(diagram, max_depth=2)
plt.savefig('drawing_graphviz_example.png')
assert os.path.exists('drawing_graphviz_example.png')

# Show the figures (but not when testing).
if 'TEST_TMPDIR' not in os.environ:
    webbrowser.open_new_tab(url='cart_pole_topology.png')
    webbrowser.open_new_tab(url='drawing_graphviz_example.png')
