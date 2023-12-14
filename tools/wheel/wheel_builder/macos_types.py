# This file contains data types used by the macOS-specific build logic. See
# //tools/wheel:builder for the user interface.

from dataclasses import dataclass
from typing import Tuple


class PythonTarget:
    def __init__(self, *version_parts):
        pv_parts = tuple(map(str, version_parts))
        self.version_tuple = tuple(*version_parts)
        self.version_full = '.'.join(pv_parts)
        self.version = '.'.join(pv_parts[:2])
        self.tag = ''.join(pv_parts[:2])
