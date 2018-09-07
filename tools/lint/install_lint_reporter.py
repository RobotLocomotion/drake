"""A helper program for install_lint.bzl that reports lint errors in a
human-readable way, with suggested fixes, etc.
"""

import argparse
import sys


def main():
    parser = argparse.ArgumentParser(
        prog='install_lint',
        description=__doc__)
    parser.add_argument(
        '--genquery_output', metavar='FILE', type=str, action='append',
        required=True, help='Required genquery report(s) to validate')
    args = parser.parse_args()

    return_code = 0

    for filename in args.genquery_output:
        with open(filename, 'r') as f:
            dependency_chain = f.readlines()
        if len(dependency_chain) > 0:
            # A dependency chain was found.  This label was correctly
            # depended-on by //:install.
            continue

        failed_label = filename.replace("/install_lint_genquery_", ":")
        print("ERROR: {} is not included in the //:install rule".format(
            failed_label))
        print("note: Check the parent folder(s) BUILD.bazel file to find out "
              "where to add a dependency")

        return_code = 1

    return return_code


if __name__ == "__main__":
    sys.exit(main())
