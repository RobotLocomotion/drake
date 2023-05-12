r"""
XXX docs!!
"""

import argparse
import logging
import numpy as np
import os
from pathlib import Path

from pydrake.common import configure_logging as _configure_logging
from pydrake.multibody._inertia_fixer import (
    InertiaFixer as _InertiaFixer,
)

_logger = logging.getLogger("drake")


def _main():
    _configure_logging()

    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "input_file", type=str,
        help="Filesystem path to an SDFormat or URDF file.")
    parser.add_argument(
        "output_file", type=str, nargs='?',
        help="[Optional] Filesystem path to write output with repaired"
        " inertias. If missing, output will go to stdout.")
    parser.add_argument(
        "--invalid_only", action="store_true",
        help="only fix physically invalid inertias.")
    parser.add_argument(
        "--in_place", action="store_true",
        help="modify the input file in-place. Any output_file argument"
        " will be ignored.")
    args = parser.parse_args()

    if 'BUILD_WORKSPACE_DIRECTORY' in os.environ:
        os.chdir(os.environ['BUILD_WORKING_DIRECTORY'])

    fixer = _InertiaFixer(**vars(args))
    fixer.fix_inertia()


if __name__ == "__main__":
    _main()
