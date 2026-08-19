# This file contains data types used by the Linux-specific build logic. See
# //tools/wheel:builder for the user interface.

from dataclasses import dataclass
from enum import Enum

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

    python_manager will be selected automatically based on _DISTRO_PYTHONS.
    """

    platform: Platform
    python: PythonTarget
    python_manager: PythonManager

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
        assert len(self.build_python.version_tuple) == 3, (
            self.build_python.version_tuple
        )
        assert isinstance(self.test_platforms, tuple)
        assert isinstance(self.test_pythons, tuple)
        for test_python in self.test_pythons:
            assert len(test_python.version_tuple) == 2, (
                test_python.version_tuple
            )

    def test_matrix(self) -> tuple[TestCase, ...]:
        """Returns the Cartesian product of `test_platforms` and
        `test_pythons`, choosing distro-provided Python (`PIP`) where
        available and falling back to `UV` otherwise."""
        result = []
        for test_platform in self.test_platforms:
            for test_python in self.test_pythons:
                python_manager = (
                    PythonManager.PIP
                    if test_python.version_tuple
                    in _DISTRO_PYTHONS[test_platform.alias]
                    else PythonManager.UV
                )
                result.append(
                    TestCase(test_platform, test_python, python_manager)
                )
        return tuple(result)
