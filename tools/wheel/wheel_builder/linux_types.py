# This file contains data types used by the Linux-specific build logic. See
# //tools/wheel:builder for the user interface.

from dataclasses import dataclass
from typing import Tuple


@dataclass
class Role:
    name: str


@dataclass
class Platform:
    name: str
    version: str
    alias: str


@dataclass
class Target:
    build_platform: Platform
    test_platform: Platform
    python_version_tuple: Tuple[int]
    python_sha: str = None

    def __post_init__(self):
        pv_parts = tuple(map(str, self.python_version_tuple))
        self.python_version_full = ".".join(pv_parts)
        self.python_version = ".".join(pv_parts[:2])
        self.python_tag = "".join(pv_parts[:2])

    def platform(self, role: Role):
        p = getattr(self, f"{role.name}_platform")
        return p if p is not None else self.build_platform


BUILD = Role("build")
TEST = Role("test")
