"""Drake's wrapper for the clang-format binary."""

import os
import sys

from python import runfiles


def get_clang_format_path():
    # In our runfiles, there should be exactly one path that looks like
    # "llvm+{module_extension_...}-{version}-{os}-{arch}/bin/clang-format".
    # Find and return it (or raise an error).
    manifest = runfiles.Create()
    matches = list(manifest.root().glob("llvm+*/bin/clang-format"))
    assert len(matches) == 1, repr(matches)
    return matches[0]


def _main():
    exe = get_clang_format_path()
    os.execvp(exe, [exe] + sys.argv[1:])


if __name__ == "__main__":
    _main()
