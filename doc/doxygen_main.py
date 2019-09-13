"""Command-line tool to generate Drake's Doxygen content.

See also drake/doc/documentation_instructions.rst.
"""

from __future__ import print_function

import argparse
from collections import OrderedDict
import os
from os.path import dirname
import shutil
import subprocess
import sys
import tempfile

from six import iteritems
from bazel_tools.tools.python.runfiles import runfiles


def _run_doxygen(drake_workspace, args):
    # Find 'doxygen' using runfiles.
    doxygen = runfiles.Create().Rlocation("doxygen/doxygen")
    assert os.path.exists(doxygen), doxygen
    # Find 'dot' using Drake's default path (for this platform).
    if sys.platform == "darwin":
        path = "/usr/local/bin:/usr/bin:/bin"
    else:
        path = "/usr/bin:/bin"
    env = {"PATH": path}
    dot = subprocess.check_output(["which", "dot"], env=env).strip()

    # Prepare the input and output folders.  We will copy the requested input
    # file(s) into a temporary scratch directory, so that Doxygen doesn't root
    # around in the drake_workspace (which is extremely slow).
    out_dir = os.path.abspath(args.out_dir)
    if not os.path.exists(out_dir):
        os.makedirs(out_dir)
    working_dir = os.path.join(drake_workspace, "doxygen_tmp")
    input_root = os.path.join(working_dir, "input")
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
        if rel_x.startswith("."):
            continue
        if rel_x.startswith("bazel"):
            continue
        if rel_x.startswith("build"):
            continue
        if rel_x.startswith("cmake"):
            continue
        if rel_x.startswith("doc"):
            # N.B. Done above.
            continue
        if rel_x.startswith("doxygen_tmp"):
            continue
        if rel_x.startswith("setup"):
            continue
        if rel_x.startswith("third_party"):
            continue
        if rel_x.startswith("tools"):
            continue

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
    definitions["OUTPUT_DIRECTORY"] = out_dir
    definitions["WARN_LOGFILE"] = os.path.join(out_dir, "doxygen.log")
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
    doxyfile = os.path.join(working_dir, "Doxyfile_CXX")
    # TODO(eric.cousineau): Import cmake_configure_file as a py_library,
    # instead of shelling out to it.
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
    subprocess.check_call([doxygen, doxyfile], cwd=working_dir)
    shutil.rmtree(working_dir)
    print("done")
    print("See file://%s/html/index.html" % out_dir)


def main():
    drake_workspace = os.path.dirname(
        runfiles.Create().Rlocation("drake/.bazelproject"))
    assert os.path.exists(drake_workspace), drake_workspace
    parser = argparse.ArgumentParser(
        description=__doc__.strip())
    parser.add_argument(
        '--quick', action='store_true', default=False,
        help="Disable slow features (e.g., all graphs).")
    parser.add_argument(
        "--out_dir", type=str, metavar="DIR", default=os.path.join(
            drake_workspace, "build/drake/doc/doxygen_cxx"),
        help="Output directory. Does not have to exist beforehand.")
    parser.add_argument(
        'inputs', nargs='*',
        help="Process only these files and/or directories; e.g., "
        "'bazel-bin/doc/doxygen --quick systems/framework' "
        "or using shell globbing, e.g., "
        "'bazel-bin/doc/doxygen --quick systems/framework/*leaf*.h'.")
    args = parser.parse_args()
    for x in args.inputs:
        if not os.path.exists(x):
            print("Inputs must be files and/or directories, but "
                  "'{}' does not exist".format(x))
            sys.exit(1)
    _run_doxygen(drake_workspace, args)


if __name__ == '__main__':
    main()
