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


def python_manager_for(
    platform: Platform, python: PythonTarget
) -> PythonManager:
    """Returns the PythonManager needed to obtain `python` on `platform`."""
    if python.version_tuple in _DISTRO_PYTHONS[platform.alias]:
        return PythonManager.PIP
    return PythonManager.UV


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
