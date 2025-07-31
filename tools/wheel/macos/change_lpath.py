"""
Given an input artifact (binary or library), apply a specified change to the
path prefix of all libraries to which the artifact is linked. The input can
also be an artifact inside a wheel.
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile

from tools.install import otool


def _extract(wheel, path):
    """
    Extracts a file from a wheel (zipfile) and returns it as a TemporaryFile.
    """
    with zipfile.ZipFile(wheel, 'r') as zf:
        tf = tempfile.NamedTemporaryFile()
        shutil.copyfileobj(zf.open(path, 'r'), tf)
        tf.flush()

        return tf


def _replace(wheel, path, content):
    """
    Replace a file in a wheel (zipfile) with the contents of a file-like
    object.
    """
    tf = tempfile.NamedTemporaryFile()

    with zipfile.ZipFile(wheel, 'r') as zold:
        with zipfile.ZipFile(tf, 'w', zipfile.ZIP_DEFLATED) as znew:
            for item in zold.infolist():
                if item.filename == path:
                    content.seek(0)
                    znew.writestr(path, content.read())
                else:
                    znew.writestr(item, zold.read(item.filename))

    os.replace(wheel, tf.name)


def _chlpath(path, libs, old, new):
    """
    Builds and applies a set of changes to a library's load paths.

    Given the set of linked libraries `libs` of the library at `path`, for each
    linked library whose path starts with `old`, replace `old` with `new`.
    """
    changes = []

    for lib in libs:
        if lib.path.startswith(old):
            changes += ['-change', lib.path, new + lib.path[len(old):]]

    if len(changes):
        subprocess.check_call(
            ['install_name_tool'] + changes + [path],
        )


def main(args):
    # Set up argument parser.
    parser = argparse.ArgumentParser(
        description='Change the prefix of dependent shared libraries.')
    parser.add_argument(
        '--old', required=True,
        help='Old prefix to be replaced')
    parser.add_argument(
        '--new', required=True,
        help='Replacement prefix')
    parser.add_argument(
        '--wheel',
        help='Modify a file inside the specified wheel.')
    parser.add_argument(
        'library', nargs='+',
        help='Dynamic library to modify')

    # Parse arguments.
    options = parser.parse_args(args)

    # Get list of dependent libraries and modify their prefixes.
    for path in options.library:
        libfile = None
        libpath = path

        if options.wheel:
            libfile = _extract(options.wheel, path)
            libpath = libfile.name

        libs = otool.linked_libraries(libpath)
        _chlpath(libpath, libs=libs, old=options.old, new=options.new)

        if options.wheel:
            _replace(options.wheel, path, libfile)


# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if __name__ == '__main__':
    main(sys.argv[1:])
