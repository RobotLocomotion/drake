"""Drake's wrapper for the clang-format binary.
"""

import os
import sys


def get_clang_format_path(version=None):
    """Call with version=None to use Drake's default version.
    Otherwise, pass the desired major version as an int.
    """
    if version is None:
        version = 9
    if sys.platform == "darwin":
        if version == 9:
            path = "/usr/local/opt/clang-format@9/bin/clang-format-9"
        elif version < 6:
            raise RuntimeError(
                f"Could not find required clang-format {version}")
        else:
            path = f"/usr/local/opt/llvm@{version}/bin/clang-format"
    else:
        if version <= 6:
            path = f"/usr/bin/clang-format-{version}.0"
        else:
            path = f"/usr/bin/clang-format-{version}"
    if os.path.isfile(path):
        return path
    raise RuntimeError("Could not find required clang-format at " + path)
