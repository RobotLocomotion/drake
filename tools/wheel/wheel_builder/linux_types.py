# This file contains data types used by the Linux-specific build logic. See
# //tools/wheel:builder for the user interface.

from dataclasses import dataclass
from enum import Enum

from .common import PythonBinder, PythonTarget


class PythonManager(Enum):
    _value_: str

    PIP = "pip"
    UV = "uv"


@dataclass
class Platform:
    name: str
    version: str
    alias: str


@dataclass
class TestCase:
    """A (platform, python) combination with which to test, along with the
    python_manager to use to obtain the requested Python version on that
    platform."""

    platform: Platform
    python: PythonTarget
    python_manager: PythonManager


@dataclass
class Target:
    python_binder: PythonBinder
    build_platform: Platform
    build_python: PythonTarget
    test_platforms: tuple[Platform, ...]
    test_pythons: tuple[PythonTarget, ...]

    def __post_init__(self):
        self.build_python.validate(n_components=3)
        assert isinstance(self.test_platforms, tuple)
        assert isinstance(self.test_pythons, tuple)
        assert self.test_pythons
        for test_python in self.test_pythons:
            test_python.validate(n_components=2)
