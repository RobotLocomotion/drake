"""Drake's wrapper for the clang-format binary.
"""

import os


def get_clang_format_path():
    preferred = "/usr/bin/clang-format-3.9"
    if os.path.isfile(preferred):
        return preferred
    return "clang-format"
