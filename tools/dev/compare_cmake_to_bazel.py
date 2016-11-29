#!/usr/bin/env python

import os
import re
import subprocess

_DRAKE_DIST = os.path.abspath(os.getcwd())

"""A manually-run tool to compare the build systems written in Bazel vs CMake.
Note that the CMake build system is specific to the configuration options that
it was built with, e.g., if SNOPT is disabled, then snopt-dependent sources
will not be identified.
"""

def _query_cmake_source_files(build_dir):
    """Returns the list of files that are source code to CMake."""
    # First, we find the *.cc files in using build.ninja.
    # Example line from ninja files:
    #     build common/CMakeFiles/drakeCommon.dir/drake_assert.cc.o: CXX_COMPILER__drakeCommon /home/jwnimmer/jwnimmer-tri/drake-distro/drake/common/drake_assert.cc
    result = set()
    with open(os.path.join(build_dir, "drake/build.ninja"), "r") as build_ninja:
        order_depends_junk = re.compile(r' \|\| cmake_order_depends\S*')
        object_target = re.compile(r'build.*\.o: CXX[^ ]* (.*)')
        for line in build_ninja:
            # Remove some junk.
            line = re.sub(order_depends_junk, '', line.strip())
            # Is it a build line?
            match = object_target.match(line)
            if match is None:
                continue
            source_file, = match.groups()
            # Make paths relative.
            source_file = source_file.replace(_DRAKE_DIST + "/", "")
            result.add(source_file)

    # Second, we find the *.h files using cmake_install.cmake.
    # Example line from cmake files.
    #    "/home/jwnimmer/jwnimmer-tri/drake-distro/drake/common/drake_assert.h"
    header_pattern = re.compile(r'^\s*"(.*)"\s*$')
    for root, dirnames, filenames in os.walk(os.path.join(build_dir, 'drake')):
        for filename in filenames:
            if filename != "cmake_install.cmake":
                continue
            with open(os.path.join(root, filename), 'r') as install_list:
                for line in install_list.readlines():
                    match =  re.match(header_pattern, line)
                    if match is None:
                        continue
                    source_file, = match.groups()
                    # Make paths relative.
                    source_file = source_file.replace(_DRAKE_DIST + "/", "")
                    result.add(source_file)

    return result


def _query_bazel_source_files():
    """Returns the list of files that are source code to Bazel."""
    # Example query output:
    #   drake-distro$ bazel query 'kind("source file", deps(//drake/...))'
    #   @spdlog//:include/spdlog/spdlog.h
    #   //drake/common:drake_throw.h
    #   //drake/common:drake_throw.cc
    # We just want the Drake sources, with : into / to make them filenames.
    output = subprocess.check_output([
        "/usr/bin/bazel", "query", "kind('source file', deps(//drake/...))"])
    result = set()
    for line in output.splitlines():
        if line.startswith("//drake"):
            result.add(line[2:].replace(':', '/'))
    return result


def report_source_files(cmake_sources, bazel_sources):
    any_set  = cmake_sources | bazel_sources
    both_set = cmake_sources & bazel_sources
    for filename in sorted([x for x in any_set]):
        ctag = "CMake" if filename in cmake_sources else "     "
        btag = "Bazel" if filename in bazel_sources else "     "
        diff = "Only" if filename not in both_set else "    "
        print diff, ctag, btag, filename


def main():
    bazel_sources = _query_bazel_source_files()
    cmake_sources = _query_cmake_source_files("build/debug")
    report_source_files(cmake_sources, bazel_sources)

if __name__ == "__main__":
    main()
