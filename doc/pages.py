"""Command-line tool to generate the Drake website contents, without the extra
reference material (C++ API, Python API, Style Guide).

For instructions, see https://drake.mit.edu/documentation_instructions.html.
"""

import os
import tempfile

from drake.doc.defs import check_call, main, symlink_input


def _build(*, out_dir, temp_dir):
    """Generates into out_dir; writes scratch files into temp_dir.
    Both directories must already exist and be empty.
    """
    assert len(os.listdir(temp_dir)) == 0
    assert len(os.listdir(out_dir)) == 0

    # Create a hermetic copy of our input.  This helps ensure that only files
    # listed in BUILD.bazel will render onto the website.
    symlink_input("drake/doc/pages_input.txt", temp_dir)

    # Run the site generator.
    check_call([
        "jekyll", "build",
        "--source", os.path.join(temp_dir, "drake/doc"),
        "--destination", out_dir,
    ])

    # The nominal pages to offer for preview.
    return ["index.html"]


if __name__ == '__main__':
    main(build=_build, subdir="", description=__doc__.strip())
