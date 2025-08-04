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


def _extract(wheel, path, dst):
    """
    Extracts a file from a wheel (zipfile) and returns it as a TemporaryFile.
    """
    with zipfile.ZipFile(wheel, 'r') as zip_file:
        file_info = zip_file.getinfo(path)

        with open(dst, 'wb') as out:
            shutil.copyfileobj(zip_file.open(path, 'r'), out)


def _replace(wheel, path, content):
    """
    Replaces a file in a wheel (zipfile) with the contents of a file-like
    object.
    """
    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp_path = os.path.join(tmp_dir, 'wheel')

        with zipfile.ZipFile(tmp_path, 'w', zipfile.ZIP_DEFLATED) as zip_new:
            with zipfile.ZipFile(wheel, 'r') as zip_old:
                for item in zip_old.infolist():
                    if item.filename == path:
                        content.seek(0)
                        zip_new.writestr(item, content.read())
                    else:
                        zip_new.writestr(item, zip_old.read(item.filename))

        os.replace(wheel, tmp_path)


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
        if options.wheel:
            with tempfile.TemporaryDirectory() as tmp_dir:
                lib_path = os.path.join(tmp_dir, 'lib')
                _extract(options.wheel, path, lib_path)

                libs = otool.linked_libraries(lib_path)
                _chlpath(lib_path, libs=libs, old=options.old, new=options.new)

                with open(lib_path, 'rb') as lib_file:
                    _replace(options.wheel, path, lib_file)

        else:
            libs = otool.linked_libraries(path)
            _chlpath(path, libs=libs, old=options.old, new=options.new)


# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if __name__ == '__main__':
    main(sys.argv[1:])
