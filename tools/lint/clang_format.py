"""Drake's wrapper for the clang-format binary.
"""

import os
import platform


def get_clang_format_path(version=None):
    """Call with version=None to use Drake's default version.
    Otherwise, pass the desired major version as an int.
    """
    if version is None:
        version = 15
    if platform.system() == "Darwin":
        if platform.machine() == "arm64":
            homebrew = "/opt/homebrew"
        else:
            homebrew = "/usr/local"
        path = f"{homebrew}/opt/llvm@{version}/bin/clang-format"
    else:
        path = f"/usr/bin/clang-format-{version}"
    if os.path.isfile(path):
        return path
    raise RuntimeError("Could not find required clang-format at " + path)
