#!/usr/bin/env python3

"""This script parses VERSION.txt (if it exists) and produces a JSON object
which specifies the variable substitutions needed for drake-config.cmake."""

import argparse
import re


def _parse_version_txt(path):
    contents = open(path, 'rt').readline()
    m = re.match(r'^[0-9.]+([+][^ ]+)?(?=\s)', contents)
    if m is None:
        raise ValueError(f'Failed to parse {path} contents')

    version_full = m.group(0)
    version_parts = version_full.split('.')
    assert len(version_parts) == 3

    return version_parts


def _write_version_info(dest, version_parts):
    with open(dest, 'wt') as f:
        if version is None:
            f.write(f'set(DRAKE_VERSION "unknown")\n')
            f.write(f'set(DRAKE_MAJOR_VERSION "unknown")\n')
            f.write(f'set(DRAKE_MAJOR_VERSION "unknown")\n')
            f.write(f'set(DRAKE_MAJOR_VERSION "unknown")\n')
        else:
            version_full = '.'.join(version_parts)
            f.write(f'set(DRAKE_VERSION "{version_full}")\n')
            f.write(f'set(DRAKE_MAJOR_VERSION "{version_parts[0]}")\n')
            f.write(f'set(DRAKE_MAJOR_VERSION "{version_parts[1]}")\n')
            f.write(f'set(DRAKE_MAJOR_VERSION "{version_parts[2]}")\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'output', type=argparse.FileType('w'),
        help='Path to output file.')
    parser.add_argument(
        'input', type=argparse.FileType('r'), nargs='?', default=None,
        help='Path to input `VERSION.TXT` (optional).')
    args = parser.parse_args()

    if args.input is not None:
        version_parts = _parse_version_txt(args.input)
    else:
        version_parts = None

    _write_version_info(args.output, version_parts

    return 0


if __name__ == '__main__':
    main()
