#!/usr/bin/env python2

"""Command-line tool to generate Drake's Doxygen content.

"""

from __future__ import print_function

import argparse
from collections import OrderedDict
import os
from os.path import dirname
import shutil
import subprocess
import sys

from six import iteritems

def _get_drake_workspace():
    """Find and return the path to the drake workspace."""

    result = dirname(dirname(os.path.abspath(sys.argv[0])))
    if not os.path.exists(os.path.join(result, "WORKSPACE")):
        raise RuntimeError("Could not place drake at " + result)
    return result

def _run_doxygen(args):
    # Find our programs.
    if sys.platform == "darwin":
        path = "/usr/local/bin:/usr/bin:/bin"
    else:
        path = "/usr/bin:/bin"
    env = {"PATH": path}
    doxygen = subprocess.check_output(["which", "doxygen"], env=env).strip()
    dot = subprocess.check_output(["which", "dot"], env=env).strip()

    # Prepare the input and output folders.  We will copy the requested input
    # file(s) into a temporary scratch directory, so that Doxygen doesn't root
    # around in the drake_workspace (which is extremely slow).
    drake_workspace = _get_drake_workspace()
    binary_dir = os.path.join(drake_workspace, "build/drake/doc")
    input_root = os.path.join(binary_dir, "input")
    if os.path.exists(input_root):
        shutil.rmtree(input_root)
    source_root = os.path.join(input_root, "drake")
    os.makedirs(source_root)
    shutil.copytree(
        os.path.join(drake_workspace, "doc"),
        os.path.join(source_root, "doc"))
    inputs = args.inputs
    if not inputs:
        # No inputs were specified; use everything.
        inputs = [
            os.path.join(drake_workspace, x)
            for x in os.listdir(drake_workspace)
        ]
    for x in inputs:
        # Find the workspace-relative pathname.
        abs_x = os.path.abspath(x)
        rel_x = os.path.relpath(abs_x, drake_workspace)
        assert not rel_x.startswith(".."), rel_x

        # Skip bad things.
        if rel_x.startswith("."): continue
        if rel_x.startswith("bazel"): continue
        if rel_x.startswith("build"): continue
        if rel_x.startswith("cmake"): continue
        if rel_x.startswith("doc"): continue  # N.B. Done above.
        if rel_x.startswith("setup"): continue
        if rel_x.startswith("third_party"): continue
        if rel_x.startswith("tools"): continue

        # Copy the workspace files into the input scratch dir.
        target = os.path.join(source_root, rel_x)
        if os.path.isfile(abs_x):
            parent = os.path.dirname(target)
            if not os.path.exists(parent):
                os.makedirs(parent)
            shutil.copy2(abs_x, target)
        else:
            assert os.path.isdir(abs_x)
            # N.B. This won't work if the user redundantly requested both a
            # parent directory and one of its children.  For now, the answer is
            # just "don't do that".
            try:
                shutil.copytree(abs_x, target)
            except OSError as e:
                print(str(e) + " during copytree.  Perhaps you tried to input "
                      "both a parent directory and its child?")
                sys.exit(1)

    # Populate the definitions dict needed by Doxygen_CXX.in.
    definitions = OrderedDict()
    definitions["INPUT_ROOT"] = input_root
    definitions["BINARY_DIR"] = binary_dir
    if args.quick:
        definitions["DOXYGEN_DOT_FOUND"] = "NO"
        definitions["DOXYGEN_DOT_EXECUTABLE"] = ""
    else:
        definitions["DOXYGEN_DOT_FOUND"] = "YES"
        definitions["DOXYGEN_DOT_EXECUTABLE"] = dot
    definition_args = ["-D%s=%s" % (key, value)
                       for key, value in iteritems(definitions)]

    # Create Doxyfile_CXX.
    in_filename = os.path.join(drake_workspace, "doc/Doxyfile_CXX.in")
    doxyfile = os.path.join(binary_dir, "Doxyfile_CXX")
    # N.B. If we executed `cmake_configure_file.py` under `bazel-bin`, it would
    # require that users do some form of `bazel build`, which would require an
    # explicit change to the doxygen building workflow.
    # TODO(eric.cousineau): Try to wrap at least bits of this in Bazel to
    # minimize this constraint.
    subprocess.check_call(
        [sys.executable,
         os.path.join(
            drake_workspace, "tools/workspace/cmake_configure_file.py"),
         "--input", in_filename,
         "--output", doxyfile,
         ] + definition_args)
    assert os.path.exists(doxyfile)

    # Run Doxygen.
    print("Building C++ Doxygen documentation...")
    sys.stdout.flush()
    subprocess.check_call([doxygen, doxyfile], cwd=binary_dir)
    shutil.rmtree(input_root)  # Don't let Bazel find the build/input copy.
    print("done")
    print("See file://%s/doxygen_cxx/html/index.html" % binary_dir)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__.strip())
    parser.add_argument(
        '--quick', action='store_true', default=False,
        help="Disable slow features (e.g., all graphs)")
    parser.add_argument(
        'inputs', nargs='*',
        help="Process only these files and/or directories; "
        "most useful using shell globbing, e.g., "
        "doxygen.py --quick systems/framework/*leaf*.h")
    args = parser.parse_args()
    _run_doxygen(args)


if __name__ == '__main__':
    main()
