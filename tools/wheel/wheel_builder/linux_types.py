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


# Python versions available via each test platform's system package manager.
_DISTRO_PYTHONS: dict[str, set[tuple[int, int]]] = {
    "AL2023": {(3, 12), (3, 13), (3, 14)},
    "noble": {(3, 12)},
    "resolute": {(3, 14)},
}


@dataclass
class Platform:
    name: str
    version: str
    alias: str


@dataclass
class TestCase:
    """A (platform, python) combination with which to test.

    python_manager is selected as PIP if the platform's system package manager
    natively provides the requested Python version, or UV otherwise.
    """

    platform: Platform
    python: PythonTarget

    def __post_init__(self):
        platform_pythons = _DISTRO_PYTHONS[self.platform.alias]
        self.python_manager = (
            PythonManager.PIP
            if self.python.version_tuple in platform_pythons
            else PythonManager.UV
        )

    @property
    def alias(self) -> str:
        return self.platform.alias


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
        for test_python in self.test_pythons:
            test_python.validate(n_components=2)

    def test_cases(self) -> tuple[TestCase, ...]:
        """Returns the Cartesian product of `test_platforms` and
        `test_pythons` as TestCases."""
        return tuple(
            itertools.starmap(
                TestCase,
                itertools.product(self.test_platforms, self.test_pythons),
            )
        )
