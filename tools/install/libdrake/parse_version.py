"""Parse the version stamp file and produce a CMake cache-style script file
which specifies the variable substitutions needed for drake-config.cmake."""

import argparse
import re

VERSION_TAG = "STABLE_VERSION"


# Check if a version string conforms to PEP 440.
def _check_version(version):
    return (
        re.match(
            r"^([1-9][0-9]*!)?(0|[1-9][0-9]*)"
            r"(\.(0|[1-9][0-9]*))*((a|b|rc)(0|[1-9][0-9]*))?"
            r"(\.post(0|[1-9][0-9]*))?(\.dev(0|[1-9][0-9]*))?"
            r"([+][a-z0-9]+([-_\.][a-z0-9]+)*)?$",
            version,
        )
        is not None
    )


# Extract full version and version parts from version stamp file.
#
# If a version is specified, the input file should contain a line starting with
# 'STABLE_VERSION', which should be three space-separated words; the tag, the
# full version, and the git SHA.
#
# This extracts the (full) version identifier, as well as the individual
# numeric parts (separated by '.') of the version. Any pre-release, 'dev',
# 'post', and/or local identifier (i.e. portion following a '+') is discarded
# when extracting the version parts. If version information is not found,
# this returns (None, None).
def _parse_stamp(stamp_file):
    # Read input.
    for line in stamp_file:
        if line.startswith(VERSION_TAG):
            tag, version_full, git_sha = line.strip().split()
            assert tag == VERSION_TAG

            # Check version format and extract numerical components.
            if not _check_version(version_full):
                raise ValueError(f"Version {version_full} is not valid")
            if re.match(r"^[1-9][0-9]*!", version_full):
                raise ValueError(
                    f"Version {version_full} contains an epoch,"
                    " which is not supported at this time"
                )

            m = re.match(r"^[0-9.]+", version_full)
            assert m

            # Check for sufficient version parts (note: user and continuous
            # builds may have more than three parts) and pad to ensure we
            # always have four.
            version_parts = m.group(0).split(".")
            if len(version_parts) < 4:
                if len(version_parts) == 3:
                    version_parts.append(0)
                else:
                    raise ValueError(
                        f"Version {version_full} does not have enough parts"
                    )

            return version_full, tuple(map(int, version_parts))

    return None, None


# Write version information to CMake cache-style script.
def _write_version_info(out, version_full, version_parts):
    if version_full is None:
        out.write('set(DRAKE_VERSION "unknown")\n')
        out.write('set(DRAKE_VERSION_MAJOR "unknown")\n')
        out.write('set(DRAKE_VERSION_MINOR "unknown")\n')
        out.write('set(DRAKE_VERSION_PATCH "unknown")\n')
        out.write('set(DRAKE_VERSION_TWEAK "unknown")\n')
    else:
        out.write(f'set(DRAKE_VERSION "{version_full}")\n')
        out.write(f'set(DRAKE_VERSION_MAJOR "{version_parts[0]}")\n')
        out.write(f'set(DRAKE_VERSION_MINOR "{version_parts[1]}")\n')
        out.write(f'set(DRAKE_VERSION_PATCH "{version_parts[2]}")\n')
        out.write(f'set(DRAKE_VERSION_TWEAK "{version_parts[3]}")\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "input",
        type=argparse.FileType("r"),
        help="Path to file optionally containing stamp version.",
    )
    parser.add_argument(
        "output", type=argparse.FileType("w"), help="Path to output file."
    )
    args = parser.parse_args()

    _write_version_info(args.output, *_parse_stamp(args.input))

    return 0


if __name__ == "__main__":
    main()
