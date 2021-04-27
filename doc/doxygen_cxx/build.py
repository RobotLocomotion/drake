"""Command-line tool to generate Drake's C++ API reference.

For instructions, see https://drake.mit.edu/documentation_instructions.html.
"""

import argparse
from fnmatch import fnmatch
import os
from os.path import join
import sys

from bazel_tools.tools.python.runfiles import runfiles

from drake.doc.defs import check_call, main, symlink_input, verbose


def _symlink_headers(*, drake_workspace, temp_dir, modules):
    """Prepare the input and output folders.  We will copy the requested input
    file(s) into a temporary scratch directory, so that Doxygen doesn't scan
    the drake_workspace directly (which is extremely slow).
    """
    # Locate the default top-level modules.
    unwanted_top_level_dirs = [
        ".*",           # There is no C++ code here.
        "bazel-*",      # Ignore Bazel build artifacts.
        "build",        # Ignore CMake build artifacts.
        "cmake",        # There is no C++ code here.
        "debian",       # Ignore Debian build artifacts.
        "doc",          # There is no C++ code here.
        "gen",          # Ignore setup artifacts.
        "setup",        # There is no C++ code here.
        "third_party",  # Only document first-party Drake code.
        "tools",        # There is no C++ code here.
        "tutorials",    # There is no C++ code here.
    ]
    default_modules = [
        f"drake.{x}" for x in os.listdir(drake_workspace)
        if os.path.isdir(join(drake_workspace, x))
        and not any([
            fnmatch(x, unwanted)
            for unwanted in unwanted_top_level_dirs
        ])
    ]

    # Iterate modules one by one.
    for module in (modules or default_modules):
        if verbose():
            print(f"Symlinking {module} ...")
        prefix = "drake."
        if not module.startswith(prefix):
            print("error: Doxygen modules must start with 'drake',"
                  f" not {module}")
            sys.exit(1)
        module_as_subdir = module[len(prefix):].replace('.', '/')
        module_workspace = join(drake_workspace, module_as_subdir)
        if not os.path.isdir(module_workspace):
            print(f"error: Unknown module {module}")
            sys.exit(1)
        for dirpath, dirs, files in os.walk(module_workspace):
            subdir = os.path.relpath(dirpath, drake_workspace)
            os.makedirs(join(temp_dir, "drake", subdir))
            for item in files:
                if any([module.startswith("drake.doc"),
                        "images" in subdir,
                        item.endswith(".h")]):
                    dest = join(temp_dir, "drake", subdir, item)
                    if not os.path.exists(dest):
                        os.symlink(join(dirpath, item), dest)


def _generate_doxyfile(*, manifest, out_dir, temp_dir, dot):
    """Creates Doxyfile_CXX from Doxyfile_CXX.in."""
    input_filename = manifest.Rlocation(
        "drake/doc/doxygen_cxx/Doxyfile_CXX.in")
    assert os.path.exists(input_filename)
    output_filename = join(temp_dir, "Doxyfile_CXX")

    cmake_configure_file = manifest.Rlocation(
        "drake/tools/workspace/cmake_configure_file")
    assert os.path.exists(cmake_configure_file)

    definitions = {}
    definitions["INPUT_ROOT"] = temp_dir
    definitions["OUTPUT_DIRECTORY"] = out_dir
    if dot:
        definitions["DOXYGEN_DOT_FOUND"] = "YES"
        definitions["DOXYGEN_DOT_EXECUTABLE"] = dot
    else:
        definitions["DOXYGEN_DOT_FOUND"] = "NO"
        definitions["DOXYGEN_DOT_EXECUTABLE"] = ""

    check_call([
        cmake_configure_file,
        "--input", input_filename,
        "--output", output_filename,
        ] + [
        "-D%s=%s" % (key, value)
        for key, value in definitions.items()
        ])
    assert os.path.exists(output_filename)
    return output_filename


def _build(*, out_dir, temp_dir, modules, quick):
    """Generates into out_dir; writes scratch files into temp_dir.
    As a precondition, both directories must already exist and be empty.
    """
    manifest = runfiles.Create()

    # Find drake's sources.
    drake_workspace = os.path.dirname(os.path.realpath(
        manifest.Rlocation("drake/.bazelproject")))
    assert os.path.exists(drake_workspace), drake_workspace
    assert os.path.exists(join(drake_workspace, "WORKSPACE")), drake_workspace

    # Find doxygen.
    doxygen = manifest.Rlocation("doxygen/doxygen")
    assert os.path.exists(doxygen), doxygen

    # Find dot.
    dot = "/usr/bin/dot"
    assert os.path.exists(dot), dot

    # Configure doxygen.
    doxyfile = _generate_doxyfile(
        manifest=manifest,
        out_dir=out_dir,
        temp_dir=temp_dir,
        dot=(dot if not quick else ""))

    # Prepare our input.
    symlink_input(
        "drake/doc/doxygen_cxx/doxygen_input.txt", temp_dir)
    _symlink_headers(
        drake_workspace=drake_workspace,
        temp_dir=temp_dir,
        modules=modules)

    # Run doxygen.
    check_call([doxygen, doxyfile], cwd=temp_dir)

    # The nominal pages to offer for preview.
    return ["", "classes.html", "modules.html"]


if __name__ == '__main__':
    main(build=_build, subdir="doxygen_cxx", description=__doc__.strip(),
         supports_modules=True, supports_quick=True)
