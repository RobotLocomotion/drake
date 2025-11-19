"""Drake's wrapper for the clang-format binary."""

import os
from pathlib import Path
import sys

from python import runfiles


def get_clang_format_path():
    manifest = runfiles.Create()
    path = Path(manifest.Rlocation("llvm/bin/clang-format"))
    if not path.is_file():
        raise RuntimeError(f"Could not find required clang-format at {path}")
    return path


def _main():
    exe = get_clang_format_path()
    os.execvp(exe, [exe] + sys.argv[1:])


if __name__ == "__main__":
    _main()
