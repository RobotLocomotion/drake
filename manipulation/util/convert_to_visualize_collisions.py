#!/usr/bin/env python3
"""
Try removing <visual> tags and turn <collision> tags into <visual> tags, for
both URDF and SDF.

@note May produce invalid XML if comments are nested within the original
<visual> tags.
"""

import argparse
import re
import sys


def replace_list(text, sub_list):
    for old, new in sub_list:
        text = re.sub(old, new, text)
    return text


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "file", type=argparse.FileType("r"), metavar="FILENAME")
    args = parser.parse_args()

    with args.file as f:
        contents = f.read()
    contents = replace_list(contents, (
        (r"<visual\b", "<!-- <visual"),
        (r"</visual>", "</visual> -->"),
        (r"<collision\b", "<visual"),
        (r"</collision>", "</visual>"),
    ))

    sys.stdout.write(contents)


if __name__ == "__main__":
    main()
