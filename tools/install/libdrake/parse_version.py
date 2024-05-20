"""Parse the version stamp file and produce a CMake cache-style script file
which specifies the variable substitutions needed for drake-config.cmake."""

import argparse
import re

VERSION_TAG = 'STABLE_VERSION'


# Extract version parts from version stamp file.
#
# If a version is specified, the input file should contain a line starting with
# 'STABLE_VERSION', which should be three space-separated words; the tag, the
# full version, and the git SHA.
#
# This extracts the individual parts (separated by '.') of the version. Any
# local identifier (i.e. portion following a '+') is removed. If version
# information is not found, this returns None.
def _parse_stamp(stamp_file):
    # Read input.
    for line in stamp_file:
        if line.startswith(VERSION_TAG):
            tag, version_full, git_sha = line.strip().split()
            assert tag == VERSION_TAG

            # Check version format.
            m = re.match(r'^[0-9.]+([+][^ ]+)?', version_full)
            if m is None:
                raise ValueError(f'Version {version_full} is not valid')

            # Check for sufficient version parts (note: user and continuous
            # builds may have more than three parts).
            version_parts = version_full.split('+')[0].split('.')
            assert len(version_parts) >= 3

            return version_parts

    return None


# Write version information to CMake cache-style script.
def _write_version_info(out, version_parts):
    if version_parts is None:
        out.write('set(DRAKE_VERSION "unknown")\n')
        out.write('set(DRAKE_VERSION_MAJOR "unknown")\n')
        out.write('set(DRAKE_VERSION_MINOR "unknown")\n')
        out.write('set(DRAKE_VERSION_PATCH "unknown")\n')
        out.write('set(DRAKE_VERSION_TWEAK "unknown")\n')
    else:
        version_full = '.'.join(version_parts)
        version_parts += [0]*4
        out.write(f'set(DRAKE_VERSION "{version_full}")\n')
        out.write(f'set(DRAKE_VERSION_MAJOR "{version_parts[0]}")\n')
        out.write(f'set(DRAKE_VERSION_MINOR "{version_parts[1]}")\n')
        out.write(f'set(DRAKE_VERSION_PATCH "{version_parts[2]}")\n')
        out.write(f'set(DRAKE_VERSION_TWEAK "{version_parts[3]}")\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'input', type=argparse.FileType('r'),
        help='Path to file optionally containing stamp version.')
    parser.add_argument(
        'output', type=argparse.FileType('w'),
        help='Path to output file.')
    args = parser.parse_args()

    _write_version_info(args.output, _parse_stamp(args.input))

    return 0


if __name__ == '__main__':
    main()
