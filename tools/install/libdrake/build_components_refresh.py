#!/usr/bin/env python

"""This script creates build_components.bzl.new with new contents based on the
current source tree.  It should be used to regularly update the version of
build_components.bzl file that is present in git."""

import os
import subprocess
import sys


def _is_dev(label):
    return "/dev:" in label or "/dev/" in label


def _remove_dev(components):
    # Filter out components that are in a dev (unsupported) package.
    return [x for x in components if not _is_dev(x)]


def _find_libdrake_components():
    query_string = ' '.join([
        'kind("cc_library",'
        'visible("//tools/install/libdrake:libdrake.so",'
        '"//drake/..."))',
        'except("//drake/examples/...")'
        'except(attr("testonly", "1", "//drake/..."))'
        'except("//drake/lcmtypes/...")'
        'except("//drake:*")'
    ])
    command = ["bazel", "query", query_string]
    components = [x for x in subprocess.check_output(command).split('\n') if x]
    components = _remove_dev(components)
    def _key(x):
        return x.split('/')
    return sorted(components, key=_key)


def main():
    mydir = os.path.abspath(os.path.dirname(sys.argv[0]))
    original_name = os.path.join(mydir, "build_components.bzl")
    new_name = os.path.join(mydir, "build_components.bzl.new")

    # Read the original version.
    with open(original_name, "r") as original:
        original_lines = original.readlines()

    # Extract the header (group 0) and footer (group 2), discarding the list of
    # components inbetween (group 1).
    header_lines = []
    footer_lines = []
    current_group = 0
    for one_line in original_lines:
        if current_group == 0:
            header_lines.append(one_line)
            if one_line == "LIBDRAKE_COMPONENTS = [\n":
                current_group = 1
            if one_line.startswith('    "//'):
                raise RuntimeError("Could not find header", header_lines)
        elif current_group == 1:
            if one_line == "]\n":
                footer_lines.append(one_line)
                current_group = 2
        elif current_group == 2:
            footer_lines.append(one_line)
        else:
            current_group = 999
    if current_group != 2:
        raise RuntimeError("Could not find header and footer", current_group)

    # Compute the new contents.
    component_labels = _find_libdrake_components()

    # Write the new version.
    with open(new_name, "w") as new:
        for one_line in header_lines:
            new.write(one_line)
        for one_label in component_labels:
            line = '    "{}",'.format(one_label)
            new.write(line)
            if len(line) > 79:
                new.write('  # noqa')
            new.write('\n')
        for one_line in footer_lines:
            new.write(one_line)

    # Done.
    return 0


if __name__ == '__main__':
    main()
