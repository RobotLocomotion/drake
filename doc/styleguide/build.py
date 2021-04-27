"""Command-line tool to generate the style guide for Drake's website.

For instructions, see https://drake.mit.edu/documentation_instructions.html.
"""

import os
from os.path import join
import textwrap

from drake.doc.defs import check_call, main, symlink_input


def _add_title(*, temp_dir, filename, title):
    """Adds a header to a Markdown file so that we can build it with Jekyll
    directly, without using the GitHub Pages infrastructure.
    The original file is replaced.
    """
    temp_dir_filename = join(temp_dir, filename)
    with open(temp_dir_filename, "r", encoding="utf-8") as f:
        data = f.read()
    os.unlink(temp_dir_filename)
    with open(temp_dir_filename, "w", encoding="utf-8") as f:
        f.write(textwrap.dedent(f"""\
            ---
            title: {title}
            ---
        """) + "\n")
        f.write(data)


def _build(*, out_dir, temp_dir):
    """Callback function that implements the bulk of main().
    Generates into out_dir; writes scratch files into temp_dir.
    As a precondition, both directories must already exist and be empty.
    """
    # Create a hermetic copy of our input.  This helps ensure that only files
    # listed in BUILD.bazel will render onto the website.
    symlink_input(
        "drake/doc/styleguide/jekyll_input.txt", temp_dir,
        strip_prefix=[
            "drake/doc/styleguide/",
            "styleguide/",
        ])

    # Prepare the files for Jekyll.
    _add_title(
        temp_dir=temp_dir,
        filename="pyguide.md",
        title="Google Python Style Guide for Drake")

    # Run the documentation generator.
    check_call([
        "/usr/bin/jekyll", "build",
        "--source", temp_dir,
        "--destination", out_dir,
    ])

    # The filenames to suggest as the starting points for preview.
    return ["cppguide.html", "pyguide.html"]


if __name__ == '__main__':
    main(build=_build, subdir="styleguide", description=__doc__.strip())
