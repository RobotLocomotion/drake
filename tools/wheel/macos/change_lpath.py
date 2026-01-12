"""
Given one or more input artifacts (binaries or libraries), apply one or more
specified changes to the path prefix of all libraries to which the artifacts
are linked.
"""

import argparse
import subprocess
import sys

from tools.install import otool


def _chlpath(path, replacements):
    """
    Builds and applies a set of changes to a library's load paths.

    Obtains the set of linked libraries of the library at `path` and, given a
    list of replacements of the form `old`, `new`, for each linked library
    whose path starts with any `old`, replaces `old` with the corresponding
    `new`. If the library has a signature, re-signs it.
    """
    changes = []

    for lib in otool.linked_libraries(path):
        for old, new in replacements:
            if lib.path.startswith(old):
                changes += ["-change", lib.path, new + lib.path[len(old) :]]

    if len(changes):
        subprocess.check_call(["install_name_tool"] + changes + [path])

        # Determine if the library needs to be re-signed.
        signature = subprocess.run(
            ["codesign", "--verify", path], capture_output=True, text=True
        )
        if not signature.stderr:
            pass  # The existing signature is valid.
        elif "code object is not signed at all" in signature.stderr:
            pass  # Not signed.
        else:
            subprocess.check_call(["codesign", "--force", "--sign", "-", path])


def main(args):
    # Set up argument parser.
    parser = argparse.ArgumentParser(
        description="Change the prefix(es) of dependent shared libraries."
    )
    parser.add_argument(
        "--old",
        required=True,
        action="append",
        help="Old prefix to be replaced (may be specified multiple times)",
    )
    parser.add_argument(
        "--new",
        required=True,
        action="append",
        help="Replacement prefix (must be specified once per --old)",
    )
    parser.add_argument(
        "library",  # BR
        nargs="+",
        help="Dynamic library to modify",
    )

    # Parse arguments.
    options = parser.parse_args(args)
    assert len(options.old) == len(options.new)
    replacements = list(zip(options.old, options.new))

    # Modify libraries.
    for path in options.library:
        _chlpath(path, replacements)


# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if __name__ == "__main__":
    main(sys.argv[1:])
