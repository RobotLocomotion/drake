#!/usr/bin/env python

"""Generate the in-order list of Drake subdirectories,
by manually parsing CMakeLists.txt.
"""

import os
import re
import sys

THIS_FILE = os.path.abspath(__file__)
UTIL_DIR = os.path.dirname(THIS_FILE)
DRAKE_DIR = os.path.dirname(UTIL_DIR)
CMAKELISTS = 'CMakeLists.txt'

def _collect(directory_path):
    result = [directory_path]
    with open(os.path.join(DRAKE_DIR, directory_path, CMAKELISTS), 'r') as f:
        for line in f.readlines():
            pattern = r'^[^#]*add_subdirectory\((.*?)\)'
            match = re.match(pattern, line, re.I)
            if match is not None:
                child, = match.groups()
                subdirectory_path = os.path.join(directory_path, child)
                if os.path.exists(os.path.join(DRAKE_DIR, subdirectory_path)):
                    result.extend(_collect(subdirectory_path))
                else:
                    print >>sys.stderr, \
                        "error: could not find", subdirectory_path
    if directory_path == "./examples":
        for child in os.listdir(os.path.join(DRAKE_DIR, directory_path)):
            subdirectory_path = os.path.join(directory_path, child)
            if os.path.isdir(os.path.join(DRAKE_DIR, subdirectory_path)):
                result.extend(_collect(subdirectory_path))
    return result


def main():
    results = _collect('.')
    for item in results:
        print item
    return 0

if __name__ == "__main__":
    main()

