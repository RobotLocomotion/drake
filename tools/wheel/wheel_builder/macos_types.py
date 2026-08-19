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
    build_python: PythonTarget
    test_pythons: tuple[PythonTarget]

    def __post_init__(self):
        assert len(self.build_python.version_tuple) == 2, (
            self.build_python.version_tuple
        )
        assert isinstance(self.test_pythons, tuple)
        for test_python in self.test_pythons:
            assert len(test_python.version_tuple) == 2, (
                test_python.version_tuple
            )
