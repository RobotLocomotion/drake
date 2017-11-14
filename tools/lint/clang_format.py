"""Drake's wrapper for the clang-format binary.
"""

import os
import sys


def get_clang_format_path():
    if sys.platform == "darwin":
        path = "/usr/local/bin/clang-format"
    else:
        path = "/usr/bin/clang-format-4.0"
    if os.path.isfile(path):
        return path
    raise RuntimeError("Could not find required clang-format at " + path)
