#!/usr/bin/env python3

"""Runs Drake Visualizer with builtin scripts under an install tree."""

from os.path import isdir, isfile, join, dirname, realpath
import sys

# Manipulate relative path, acommodating the potential use of symlinks.
PREFIX_DIR = dirname(dirname(realpath(__file__)))
assert isdir(join(PREFIX_DIR, "bin")), f"Bad location: {PREFIX_DIR}"
# This is adapted from a sample virtualenv's `activate_this.py` script.
sys.path.insert(
    0, join(PREFIX_DIR, f"lib/python{sys.version[:3]}/site-packages"))

from _drake_visualizer_builtin_scripts import (  # noqa
    _exec_drake_visualizer_with_plugins,
)


def main():
    # Execute wrapper.
    _exec_drake_visualizer_with_plugins(
        drake_visualizer_real=join(PREFIX_DIR, "bin/drake-visualizer-real"),
        # Ensure the wrapped binary shows 'drake-visualizer' in its usage.
        arg0=__file__,
    )


assert __name__ == "__main__"
main()
