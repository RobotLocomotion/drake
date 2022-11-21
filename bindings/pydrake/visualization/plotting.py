"""This module is deprecated. Please import its functions directly from the
parent module (pydrake.visualization) instead.
"""

from pydrake.common.deprecation import deprecated_callable as _deprecated
import pydrake.visualization as _new_home


@_deprecated(
    "The pydrake.visualization.plotting module is deprecated. Please import "
    "its functions directly from the parent module (pydrake.visualization) "
    "instead.",
    date="2023-02-01")
def plot_sublevelset_quadratic(*args, **kwargs):
    _new_home.plot_sublevelset_quadratic(*args, **kwargs)


@_deprecated(
    "The pydrake.visualization.plotting module is deprecated. Please import "
    "its functions directly from the parent module (pydrake.visualization) "
    "instead.",
    date="2023-02-01")
def plot_sublevelset_expression(*args, **kwargs):
    _new_home.plot_sublevelset_expression(*args, **kwargs)
