"""Applies fixes to protobuf-generated files to address ubsan errors. """

import re
import sys


def fixup(src, out):
    insert_after = '#include <google/protobuf/generated_message_reflection.h>'
    insert_text = '\n#include "drake/common/proto/protobuf-ubsan-fixup.h"'

    with open(src, 'r') as fsrc, open(out, 'w') as fout:
        text = fsrc.read()
        # Ensure local includes are prefixed with 'drake'.
        text = re.sub(
            r'^#include "',
            r'#include "drake/',
            text, flags=re.MULTILINE)
        # Insert fixup header.
        text = re.sub(
            '(' + re.escape(insert_after) + ')',
            r'\1' + insert_text,
            text)
        fout.write(text)


if __name__ == "__main__":
    assert len(sys.argv) == 3
    # Simplify argument parsing by taking all files separated by spaces.
    srcs = sys.argv[1].split()
    outs = sys.argv[2].split()
    assert len(srcs) == len(outs), (
        "Must supply outputs in the same order and number as outputs")
    for src, out in zip(srcs, outs):
        fixup(src, out)
