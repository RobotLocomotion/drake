"""Drake's wrapper for the clang-format binary.
"""

import os
import platform


def get_clang_format_path(version=None):
    """Call with version=None to use Drake's default version.
    Otherwise, pass the desired major version as an int.
    """
    if version is None:
        version = 9
    if platform.system() == "Darwin":
        if platform.machine() == "arm64":
            homebrew = "/opt/homebrew"
        else:
            homebrew = "/usr/local"

        if version == 9:
            path = f"{homebrew}/opt/clang-format@9/bin/clang-format-9"
        elif version < 6:
            raise RuntimeError(
                f"Could not find required clang-format {version}")
        else:
            path = f"{homebrew}/opt/llvm@{version}/bin/clang-format"
    else:
        if version <= 6:
            path = f"/usr/bin/clang-format-{version}.0"
        else:
            path = f"/usr/bin/clang-format-{version}"
    if os.path.isfile(path):
        return path
    raise RuntimeError("Could not find required clang-format at " + path)
