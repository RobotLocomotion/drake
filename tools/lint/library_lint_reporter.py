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
        "--missing-deps",
        metavar="FILE",
        type=str,
        help=(
            "Filename containing list of labels that are missing. "
            "It is a lint error when this file is non-empty."
        ),
    )
    parser.add_argument(
        "--extra-deps",
        metavar="FILE",
        type=str,
        help=(
            "Filename containing list of labels that are extra. "
            "It is a lint error when this file is non-empty."
        ),
    )
    parser.add_argument(
        "--untagged-package-library",
        action="store_true",
        default=False,
        help=(
            "A cc_library exists with the short_package_name but it "
            "does not use drake_cc_package_library"
        ),
    )
    args = parser.parse_args()
    build_file_name = args.package_name[2:] + "/BUILD.bazel"
    short_package_name = args.package_name.split("/")[-1]

    return_code = 0

    if args.untagged_package_library:
        print(
            f"ERROR: The package {args.package_name} has a "
            f'cc_library(name = ":{short_package_name}") '
            "declared without using drake_cc_package_library()."
        )
        return_code = 1

    with open(args.missing_deps or "/dev/null") as opened:
        missing_deps = opened.readlines()
    if missing_deps:
        print(
            f"ERROR: Missing deps in {args.package_name}'s "
            "drake_cc_package_library."
        )
        print(
            f"note: In the {build_file_name} rule for "
            "drake_cc_package_library, add the following lines to the deps:"
        )
        for dep in sorted(missing_deps):
            print(f'        ":{dep.strip().split(":")[-1]}",')
        print(
            "note: Alternatively, if some of these libraries should not be "
            "added to the drake_cc_package_library, you may tag them with "
            f'"{_TAG_EXCLUDE_FROM_PACKAGE}" in order to explicitly exclude '
            "them."
        )
        return_code = 1

    with open(args.extra_deps or "/dev/null") as opened:
        extra_deps = opened.readlines()
    extra_deps = [
        # Filter out false positives.  All C++ code is OK to depend on these.
        item
        for item in extra_deps
        if not (
            item.startswith("//tools/cc_toolchain:") or "@bazel_tools//" in item
        )
    ]
    if extra_deps:
        print(
            f"ERROR: Extra deps in {args.package_name}'s "
            "drake_cc_package_library."
        )
        print(
            f"note: In the {build_file_name} rule for "
            "drake_cc_package_library, remove the following lines from the "
            "deps:"
        )
        for dep in sorted(extra_deps):
            print(f'        "{dep.strip()}",')
        return_code = 1

    return return_code


if __name__ == "__main__":
    sys.exit(main())
