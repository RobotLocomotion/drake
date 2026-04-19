#!/usr/bin/env python3
"""kcoverage.py -- mimic coverage.py[1] just enough to shim kcov into bazel
coverage builds.

[1]: https://pypi.org/project/coverage/
"""

import argparse
import os
import subprocess
import sys


def parse_run_command(argv):
    """Absorb the coverage.py 'run' command line. Drake will ignore most of it.
    Returns a list of un-parsed arguments to the target executable.
    """
    parser = argparse.ArgumentParser(prog="kcoverage.py run")
    ignore = "Drake will ignore this coverage.py argument."
    parser.add_argument("-a", "--append", action="store_true", help=ignore)
    parser.add_argument("--branch", action="store_true", help=ignore)
    parser.add_argument("--rcfile", type=str, help=ignore)

    return parser.parse_known_args(argv)[1]


def exec_with_kcov(exe, exe_args):
    """Run exe with exe_args under kcov, getting the kcov command line prefix
    from the DRAKE_KCOV_COMMAND environment variable.
    """
    kcov_command = os.environ["DRAKE_KCOV_COMMAND"]

    command = (
        [x.replace('"', "") for x in kcov_command.split()] + [exe] + exe_args
    )
    result = subprocess.call(command)
    sys.exit(result)


def main():
    verb = sys.argv[1]
    if verb == "lcov":
        # Bazel 6 invokes 'lcov', but drake/kcov don't need to do anything.
        return
    # Drake/kcov support only needs to implement 'run'; anything else that
    # reached here is an error.
    assert verb == "run", f"Command '{verb}' not implemented."
    extra = parse_run_command(sys.argv[2:])
    exec_with_kcov(extra[0], extra[1:])


if __name__ == "__main__":
    main()
