# This file contains data types used by the Linux-specific build logic. See
# //tools/wheel:builder for the user interface.

from dataclasses import dataclass
from enum import Enum


class PythonManager(Enum):
    _value_: str

    PIP = "pip"
    UV = "uv"


@dataclass
class Role:
    name: str


@dataclass
class Platform:
    name: str
    version: str
    alias: str
    python_manager: PythonManager = PythonManager.PIP


@dataclass
class Target:
    build_platform: Platform
    test_platforms: tuple[Platform]
    python_version_tuple: tuple[int]
    python_sha: str = None

    def __post_init__(self):
        assert isinstance(self.test_platforms, tuple)
        pv_parts = tuple(map(str, self.python_version_tuple))
        self.python_version_full = ".".join(pv_parts)
        self.python_version = ".".join(pv_parts[:2])
        self.python_tag = "".join(pv_parts[:2])

    def platform(self, role: Role, test_index: int | None = None) -> Platform:
        """Returns the Platform for the given `role`. For the test role, the
        `test_index` into the `self.test_platforms` tuple is required. For the
        build role, the `test_index` must be None."""
        if role.name == "build":
            assert test_index is None
            return self.build_platform
        if role.name == "test":
            assert test_index is not None
            return self.test_platforms[test_index]
        raise NotImplementedError(role.name)


BUILD = Role("build")
TEST = Role("test")
