#!/usr/bin/env python

"""Command-line tool to generate Drake's Doxygen content.

"""

import argparse
import os
import subprocess
import sys

from collections import OrderedDict
from os.path import dirname

def _get_drake_distro():
    """Find and return the path to the drake workspace."""

    result = dirname(dirname(os.path.abspath(sys.argv[0])))
    if not os.path.exists(os.path.join(result, "WORKSPACE")):
        raise RuntimeError("Could not place drake at " + result)
    return result

def _run_doxygen(args):
    # Find our programs.
    doxygen = subprocess.check_output(["which", "doxygen"]).strip()
    dot = subprocess.check_output(["which", "dot"]).strip()

    # Compute the definitions dict needed by Doxygen_CXX.in.
    drake_distro = _get_drake_distro()
    binary_dir = os.path.join(drake_distro, "build/drake/doc")
    if not os.path.exists(binary_dir):
        os.makedirs(binary_dir)
    definitions = OrderedDict()
    definitions["DRAKE_WORKSPACE_ROOT"] = drake_distro
    definitions["BINARY_DIR"] = binary_dir
    if args.quick:
        definitions["DOXYGEN_DOT_FOUND"] = "NO"
        definitions["DOXYGEN_DOT_EXECUTABLE"] = ""
    else:
        definitions["DOXYGEN_DOT_FOUND"] = "YES"
        definitions["DOXYGEN_DOT_EXECUTABLE"] = dot
    definition_args = ["-D%s=%s" % (key, value)
                       for key, value in definitions.iteritems()]

    # Create Doxyfile_CXX.
    in_filename = os.path.join(drake_distro, "doc/Doxyfile_CXX.in")
    doxyfile = os.path.join(drake_distro, "build/drake/doc/Doxyfile_CXX")
    subprocess.check_call(
        [os.path.join(
            _get_drake_distro(), "tools/workspace/cmake_configure_file.py"),
         "--input", in_filename,
         "--output", doxyfile,
         ] + definition_args)
    assert os.path.exists(doxyfile)

    # Run Doxygen.
    print "Building C++ Doxygen documentation...",
    sys.stdout.flush()
    subprocess.check_call([doxygen, doxyfile], cwd=binary_dir)
    print "done"
    print "See file://%s/doxygen_cxx/html/index.html" % binary_dir


def main():
    parser = argparse.ArgumentParser(
        description=__doc__.strip())
    parser.add_argument(
        '--quick', action='store_true', default=False,
        help="Disable slow features (e.g., all graphs)")
    args = parser.parse_args()
    _run_doxygen(args)


if __name__ == '__main__':
    main()
