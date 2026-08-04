"""
Remove RPATH commands from an artifact (binary or library).
"""

import argparse
import subprocess
import sys

from tools.install import otool


def _filter_rpaths(paths, exclusions):
    """
    Returns `paths`, less any items that start with any of `exclusions`.
    """

    def _filter(path):
        for x in exclusions:
            if path.startswith(x):
                return False
        return True

    return list(filter(_filter, paths))


def _strip_rpaths(path, rpaths):
    """
    Removes RPATHs `rpaths` from binary/library `path`.
    """
    for rpath in rpaths:
        subprocess.check_call(
            ["install_name_tool", "-delete_rpath", rpath, path],
        )


def main(args):
    # Set up argument parser.
    parser = argparse.ArgumentParser(
        description="Strip RPATH(s) from a library."
    )
    parser.add_argument(
        "-x",
        "--exclude",
        metavar="PREFIX",
        action="append",
        help="leave any RPATH that starts with the specified prefix",
    )
    parser.add_argument(
        "library",  # BR
        help="dynamic library to modify",
    )

    # Parse arguments.
    options = parser.parse_args(args)

    # Extract RPATH entries from load commands.
    commands = otool.load_commands(options.library)
    rpaths = _filter_rpaths(
        paths=[c["path"] for c in commands if c["cmd"] == "LC_RPATH"],
        exclusions=options.exclude,
    )

    # Remove the RPATH load commands.
    _strip_rpaths(options.library, rpaths)


# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if __name__ == "__main__":
    main(sys.argv[1:])
