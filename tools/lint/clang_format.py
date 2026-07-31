"""Drake's wrapper for the clang-format binary."""

import os
import sys

from python import runfiles


def get_clang_format_path():
    manifest = runfiles.Create()
    path = next(manifest.root().glob("llvm*/bin/clang-format"), None)
    if path is None or not path.is_file():
        raise RuntimeError(f"Could not find required clang-format at {path}")
    return path


def _main():
    exe = get_clang_format_path()
    os.execvp(exe, [exe] + sys.argv[1:])


if __name__ == "__main__":
    _main()
