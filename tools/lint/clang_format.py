"""Drake's wrapper for the clang-format binary.
"""

import os
import platform


def get_clang_format_path(version=None):
    """Call with version=None to use Drake's default version.
    Otherwise, pass the desired major version as an int.
    """
    if version is None:
        version = 12

    name = f"clang-format-{version}"

    if platform.system() == "Darwin":
        if platform.machine() == "arm64":
            homebrew = "/opt/homebrew"
        else:
            homebrew = "/usr/local"

        path = f"{homebrew}/opt/clang-format@{version}/bin/{name}"
    else:
        path = f"/usr/bin/{name}"
    if os.path.isfile(path):
        return path
    raise RuntimeError("Could not find required clang-format at " + path)
