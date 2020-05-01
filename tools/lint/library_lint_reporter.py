"""A helper program for library_lint.bzl that reports lint errors in a
human-readable way, with suggested fixes, etc.
"""

import argparse
import sys

# Keep this constant in sync with library_lint.bzl.
_TAG_EXCLUDE_FROM_PACKAGE = "exclude_from_package"


def main():
    parser = argparse.ArgumentParser(
        prog='library_lint',
        description=__doc__)
    parser.add_argument(
        '--package-name', metavar='LABEL', type=str, required=True,
        help='Required name of package, e.g., //common/test_utilities')
    parser.add_argument(
        '--missing-deps', metavar='FILE', type=str,
        help=('Filename containing list of labels that are missing. '
              'It is a lint error when this file is non-empty.'))
    parser.add_argument(
        '--extra-deps', metavar='FILE', type=str,
        help=('Filename containing list of labels that are extra. '
              'It is a lint error when this file is non-empty.'))
    parser.add_argument(
        '--untagged-package-library', action='store_true', default=False,
        help=('A cc_library exists with the short_package_name but it '
              'does not use drake_cc_package_library'))
    args = parser.parse_args()
    build_file_name = args.package_name[2:] + "/BUILD.bazel"
    short_package_name = args.package_name.split("/")[-1]

    return_code = 0

    if args.untagged_package_library:
        print(("ERROR: The package {} has a cc_library(name = \":{}\") "
               "declared without using drake_cc_package_library().").format(
            args.package_name,
            short_package_name))
        return_code = 1

    with open(args.missing_deps or '/dev/null') as opened:
        missing_deps = opened.readlines()
    if missing_deps:
        print(("ERROR: Missing deps in {}'s drake_cc_package_library.").format(
            args.package_name))
        print(("note: In the {} rule for drake_cc_package_library, "
               "add the following lines to the deps:").format(
                   build_file_name))
        for dep in sorted(missing_deps):
            print("        \":{}\",".format(dep.strip().split(":")[-1]))
        print(("note: Alternatively, if some of these libraries should not be "
               "added to the drake_cc_package_library, you may tag them with "
               "\"{}\" in order to explicitly exclude them.").format(
                   _TAG_EXCLUDE_FROM_PACKAGE))
        return_code = 1

    with open(args.extra_deps or '/dev/null') as opened:
        extra_deps = opened.readlines()
    extra_deps = [
        # Filter out false positives.  All C++ code is OK to depend on these.
        item for item in extra_deps
        if not (item.startswith("//tools/cc_toolchain:")
                or item.startswith("@bazel_tools//"))
    ]
    if extra_deps:
        print(("ERROR: Extra deps in {}'s drake_cc_package_library.").format(
            args.package_name))
        print(("note: In the {} rule for drake_cc_package_library, "
               "remove the following lines from the deps:").format(
                   build_file_name))
        for dep in sorted(extra_deps):
            print("        \"{}\",".format(dep.strip()))
        return_code = 1

    return return_code


if __name__ == "__main__":
    sys.exit(main())
