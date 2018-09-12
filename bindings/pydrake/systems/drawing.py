"""
Provides general visualization utilities. This is NOT related to `rendering`.
@note This is an optional module, dependent on `pydot` and `matplotlib` being
installed.
"""

from StringIO import StringIO

import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import pydot


# TODO(eric.cousineau): Move `plot_graphviz` to something more accessible to
# `call_python_client`.


def plot_graphviz(dot_text):
    """Renders a DOT graph in matplotlib."""
    # @ref https://stackoverflow.com/a/18522941/7829525
    # Tried (reason ignored): pydotplus (`pydot` works), networkx
    # (`read_dot` does not work robustly?), pygraphviz (coupled with
    # `networkx`).
    g = pydot.graph_from_dot_data(dot_text)
    if isinstance(g, list):
        # Per Ioannis's follow-up comment in the above link, in pydot >= 1.2.3
        # `graph_from_dot_data` returns a list of graphs.
        # Handle this case for now.
        assert len(g) == 1
        g = g[0]
    s = StringIO()
    g.write_png(s)
    s.seek(0)
    plt.axis('off')
    return plt.imshow(plt.imread(s), aspect="equal")


def plot_system_graphviz(system):
    """Renders a System's Graphviz representation in `matplotlib`. """
    return plot_graphviz(system.GetGraphvizString())
