"""Drake's wrapper for the clang-format binary.
"""

import os
import sys


def get_clang_format_path():
    if sys.platform == "darwin":
        preferred = "/usr/local/bin/clang-format"
    else:
        preferred = "/usr/bin/clang-format-3.9"
    if os.path.isfile(preferred):
        return preferred
    return "clang-format"
