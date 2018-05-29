#!/usr/bin/env python2

"""This script creates build_components.bzl.new with new contents based on the
current source tree.  It should be used to regularly update the version of
build_components.bzl file that is present in git."""

import argparse
import os
import subprocess
import sys


def _is_dev(label):
    return "/dev:" in label or "/dev/" in label


def _remove_dev(components):
    # Filter out components that are in a dev (unsupported) package.
    return [x for x in components if not _is_dev(x)]


def _label_sort_key(label):
    # How to compare labels (lexicographically by subpackage names).
    return label.split("/")


def _is_full_package_library(one_label):
    package, short_name = one_label.split(":")
    if package.endswith("/" + short_name):
        return package
    else:
        return None


def _bazel_query(args):
    output = subprocess.check_output(["bazel", "query"] + args)
    return [x for x in output.split('\n') if x]


def _find_libdrake_components():
    # This forms the set of cc_library targets that will be installed.
    components_query = """
kind("cc_library", visible("//tools/install/libdrake:libdrake.so", "//..."))
    except(attr("testonly", "1", "//..."))
    except("//:*")
    except("//examples/...")
    except("//lcmtypes/...")
    except("//tools/install/libdrake:*")
    except(attr(tags, "exclude_from_libdrake", //...))
"""
    # First, find the drake_cc_package_library targets within that query.
    package_libs = []
    for label in _bazel_query([
            'attr(tags, "{}", {})'.format(
                "drake_cc_package_library", components_query)]):
        new_label = _is_full_package_library(label)
        assert new_label
        package_libs.append(new_label)
    # Then, find any remaining cc_library targets that are not part of a
    # drake_cc_package_library.
    misc_libs = _bazel_query([
        components_query + " ".join([
            "except deps({}, 1)".format(x)
            for x in package_libs
        ])])
    misc_libs = _remove_dev(misc_libs)
    # Sort the result for consistency.
    return sorted(package_libs + misc_libs, key=_label_sort_key)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-f", "--force", action="store_true",
        help="Overwrite existing `build_components.bzl`, rather than write " +
             "to `build_components.bzl.new`.")
    args = parser.parse_args()

    mydir = os.path.abspath(os.path.dirname(sys.argv[0]))
    original_basename = "build_components.bzl"
    original_name = os.path.join(mydir, original_basename)

    if not args.force:
        new_basename = original_basename + ".new"
    else:
        new_basename = original_basename
    new_name = os.path.join(mydir, new_basename)

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
            if ":" in one_label:
                line += '  # unpackaged'
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
