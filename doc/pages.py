"""Command-line tool to generate the Drake website contents, without the extra
reference material (C++ API, Python API, Style Guide).

For instructions, see https://drake.mit.edu/documentation_instructions.html.
"""

import os
import tempfile

from drake.doc.defs import check_call, main, symlink_input


def _build(*, out_dir, temp_dir):
    """Callback function that implements the bulk of main().
    Generates into out_dir; writes scratch files into temp_dir.
    As a precondition, both directories must already exist and be empty.
    """
    # Create a hermetic copy of our input.  This helps ensure that only files
    # listed in BUILD.bazel will render onto the website.
    symlink_input("drake/doc/pages_input.txt", temp_dir)

    # Run the documentation generator.
    check_call([
        "jekyll", "build",
        "--source", os.path.join(temp_dir, "drake/doc"),
        "--destination", out_dir,
    ])

    # The filename to suggest as the starting point for preview; in this case,
    # it's an empty filename (i.e., the index page).
    return [""]


if __name__ == '__main__':
    main(build=_build, subdir="", description=__doc__.strip())
