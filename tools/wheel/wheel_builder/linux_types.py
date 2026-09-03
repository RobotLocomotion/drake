# This file contains data types used by the Linux-specific build logic. See
# //tools/wheel:builder for the user interface.

from dataclasses import dataclass
from enum import Enum
import itertools

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

    def test_cases(
        self, distro_pythons: dict[str, set[tuple[int, int]]]
    ) -> tuple[TestCase, ...]:
        """Returns the Cartesian product of `test_platforms` and
        `test_pythons` as a tuple of `TestCase` instances.

        The `python_manager` is chosen as PIP if the given Python version is
        available via the given system's package manager, or UV otherwise.
        """

        def _make_test_case(
            platform: Platform, python: PythonTarget
        ) -> TestCase:
            python_manager = (
                PythonManager.PIP
                if python.version_tuple in distro_pythons[platform.alias]
                else PythonManager.UV
            )
            return TestCase(platform, python, python_manager)

        return tuple(
            itertools.starmap(
                _make_test_case,
                itertools.product(self.test_platforms, self.test_pythons),
            )
        )
