"""Command-line tool to generate the style guide for Drake's website.

For instructions, see https://drake.mit.edu/documentation_instructions.html.
"""

import os
from os.path import join
import textwrap

from drake.doc.defs import check_call, main, symlink_input


def _build(*, out_dir, temp_dir):
    """Generates into out_dir; writes scratch files into temp_dir.
    Both directories must already exist and be empty.
    """
    # Create a hermetic copy of our input.  This helps ensure that only files
    # listed in BUILD.bazel will render onto the website.
    symlink_input(
        "drake/doc/styleguide/jekyll_input.txt", temp_dir,
        strip_prefix=[
            "drake/doc/styleguide/",
            "styleguide/",
        ])

    # Add a header to the pyguide so that we can build it with Jekyll directly,
    # without using the GitHub Pages infrastructure.
    pyguide_md = join(temp_dir, "pyguide.md")
    with open(pyguide_md, "r", encoding="utf-8") as f:
        pyguide = f.read()
    os.remove(pyguide_md)
    with open(pyguide_md, "w", encoding="utf-8") as f:
        f.write(textwrap.dedent("""\
            ---
            title: Google Python Style Guide for Drake
            ---
        """) + "\n")
        f.write(pyguide)

    # Run the documentation generator.
    check_call([
        "/usr/bin/jekyll", "build",
        "--source", temp_dir,
        "--destination", out_dir,
    ])

    # The nominal pages to offer for preview.
    return ["cppguide.html", "pyguide.html"]


if __name__ == '__main__':
    main(build=_build, subdir="styleguide", description=__doc__.strip())
