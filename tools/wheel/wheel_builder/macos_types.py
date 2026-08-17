# This file contains data types used by the macOS-specific build logic. See
# //tools/wheel:builder for the user interface.

from dataclasses import dataclass

from .common import PythonBinder, PythonTarget


@dataclass
class Target:
    """
    A representation of a build target, constructed from the binder and Python
    target.

    Example:
        Target(PythonBinder.NANOBIND, PythonTarget(3, 2, 1))

    Members:
        python_binder: Which binder to use.
        python: Which version of Python to use.
    """
    python_binder: PythonBinder
    python: PythonTarget
