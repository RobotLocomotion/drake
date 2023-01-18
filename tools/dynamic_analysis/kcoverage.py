#!/usr/bin/env python3
"""kcoverage.py -- mimic coverage.py[1] just enough to shim kcov into bazel
coverage builds.

[1]: https://pypi.org/project/coverage/
"""

import argparse
import os
import subprocess
import sys


def parse_command_line(argv):
    """Absorb the coverage.py command line. We may ignore most of it.  Returns
    a pair of an argparse.Namespace object, and a list of un-parsed arguments
    to the target executable.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('verb', type=str, help='coverage.py verb')
    parser.add_argument('-a', '--append', action='store_true', default=False,
                        help='coverage.py --append')
    parser.add_argument('--branch', action='store_true', default=False,
                        help='coverage.py --branch')
    parser.add_argument('--rcfile', type=str, help='coverage.py rcfile')
    parser.add_argument('-o', type=str, help='coverage.py outfile')

    return parser.parse_known_args(argv)


def exec_with_kcov(exe, exe_args):
    """Run exe with exe_args under kcov, getting the kcov command line prefix
    from the DRAKE_KCOV_COMMAND environment variable.
    """
    kcov_command = os.environ['DRAKE_KCOV_COMMAND']

    # Various pieces of drake machinery need the full Bazel symlink path in
    # order to find other assets, but kcov is confused by symlinks (see
    # kcov#368). Work around kcov#368 by exporting the original exe path in
    # DRAKE_KCOV_LINK_PATH and handing the real path to kcov.
    os.environ["DRAKE_KCOV_LINK_PATH"] = exe
    exe = os.path.realpath(exe)

    command = (
        [x.replace('"', '') for x in kcov_command.split()]
        + [exe] + exe_args)
    result = subprocess.call(command)
    sys.exit(result)


def main():
    args, extra = parse_command_line(sys.argv[1:])
    if args.verb != 'run':
        return
    exec_with_kcov(extra[0], extra[1:])


if __name__ == "__main__":
    main()
