"""Drake's wrapper for the clang-format binary.
"""

import os
import sys


def get_clang_format_path(version=None):
    """Call with version=None to use Drake's default version.
    Otherwise, pass the desired major verison as an int.
    """
    if version is None:
        version = 9
    if sys.platform == "darwin":
        path = f"/usr/local/opt/llvm@{version}/bin/clang-format"
    else:
        if version <= 6:
            path = f"/usr/bin/clang-format-{version}.0"
        else:
            path = f"/usr/bin/clang-format-{version}"
    if os.path.isfile(path):
        return path
    raise RuntimeError("Could not find required clang-format at " + path)
