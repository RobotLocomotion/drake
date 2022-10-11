"""
Provides general visualization utilities. This is NOT related to `rendering`.
"""

from tempfile import NamedTemporaryFile

from pydrake.common import temp_directory

# TODO(eric.cousineau): Move `plot_graphviz` to something more accessible to
# `call_python_client`.


def _plt():
    """Deferred import of plt, so pydrake.all importing doesn't bomb out."""
    # On Ubuntu the Debian package python3-tk is a recommended (but not
    # required) dependency of python3-matplotlib; help users understand
    # that by providing a nicer message upon a failure to import.
    result = None
    try:
        import matplotlib.pyplot as __plt
        result = __plt
    except ImportError as e:
        if e.name == 'tkinter':
            result = None
        else:
            raise
    if result is None:
        raise NotImplementedError(
            "On Ubuntu when using the default pyplot configuration (i.e.,"
            " the TkAgg backend) you must 'sudo apt install python3-tk' to"
            " obtain Tk support. Alternatively, you may set MPLBACKEND to"
            " something else (e.g., Qt5Agg).")
    return result


def _pydot():
    """Deferred import of pyplot, so pydrake.all importing doesn't bomb out."""
    import pydot as __pydot
    return __pydot


def plot_graphviz(dot_text):
    """Renders a DOT graph in matplotlib."""
    plt = _plt()
    pydot = _pydot()

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
    f = NamedTemporaryFile(suffix='.png', dir=temp_directory())
    g.write_png(f.name)
    plt.axis('off')

    return plt.imshow(plt.imread(f.name), aspect="equal")


def plot_system_graphviz(system, **kwargs):
    """Renders a System's Graphviz representation in `matplotlib`."""
    return plot_graphviz(system.GetGraphvizString(**kwargs))
