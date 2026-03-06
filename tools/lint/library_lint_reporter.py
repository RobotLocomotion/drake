"""A helper program for library_lint.bzl that reports lint errors in a
human-readable way, with suggested fixes, etc.
"""

import argparse
import sys

# Keep this constant in sync with library_lint.bzl.
_TAG_EXCLUDE_FROM_PACKAGE = "exclude_from_package"


def main():
    parser = argparse.ArgumentParser(prog="library_lint", description=__doc__)
    parser.add_argument(
        "--package-name",
        metavar="LABEL",
        type=str,
        required=True,
        help="Required name of package, e.g., //common/test_utilities",
    )
    parser.add_argument(
        "--missing",
        metavar="LABEL",
        type=str,
        action="append",
        help="One label that is missing from the package library.",
    )
    parser.add_argument(
        "--extra",
        metavar="LABEL",
        type=str,
        action="append",
        help="One label that is extra in the package library.",
    )
    args = parser.parse_args()
    build_file_name = args.package_name[2:] + "/BUILD.bazel"

    return_code = 0

    if args.missing:
        print(
            f"ERROR: Missing deps in {args.package_name}'s "
            "drake_cc_package_library."
        )
        print(
            f"note: In the {build_file_name} rule for "
            "drake_cc_package_library, add the following lines to the deps:"
        )
        for dep in sorted(args.missing):
            print(f'        "{dep}",')
        print(
            "note: Alternatively, if some of these libraries should not be "
            "added to the drake_cc_package_library, you may tag them with "
            f'"{_TAG_EXCLUDE_FROM_PACKAGE}" in order to explicitly exclude '
            "them."
        )
        return_code = 1

    if args.extra:
        print(
            f"ERROR: Extra deps in {args.package_name}'s "
            "drake_cc_package_library."
        )
        print(
            f"note: In the {build_file_name} rule for "
            "drake_cc_package_library, remove the following lines from the "
            "deps:"
        )
        for dep in sorted(args.extra):
            print(f'        "{dep}",')
        return_code = 1

    return return_code


if __name__ == "__main__":
    sys.exit(main())
