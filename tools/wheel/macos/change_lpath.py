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


def _replace(wheel, files):
    """
    Replaces file(s) in a wheel (zipfile).

    Given an input dictionary mapping zip-file paths to on-disk paths, replace
    matching files in the specified wheel with the content of the respective
    on-disk files.
    """
    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp_path = os.path.join(tmp_dir, 'wheel')

        with zipfile.ZipFile(tmp_path, 'w', zipfile.ZIP_DEFLATED) as zip_new:
            with zipfile.ZipFile(wheel, 'r') as zip_old:
                for item in zip_old.infolist():
                    replacement = files.get(item.filename)
                    if replacement is not None:
                        zip_new.writestr(item, open(replacement, 'rb').read())
                    else:
                        zip_new.writestr(item, zip_old.read(item.filename))

        os.replace(tmp_path, wheel)


def _chlpath(path, replacements):
    """
    Builds and applies a set of changes to a library's load paths.

    Obtain the set of linked libraries of the library at `path` and, given a
    list of replacements of the form `old`, `new`, for each linked library
    whose path starts with any `old`, replace `old` with the corresponding
    `new`. If the library has a signature, re-sign it.
    """
    changes = []

    for lib in otool.linked_libraries(path):
        for old, new in replacements:
            if lib.path.startswith(old):
                changes += ['-change', lib.path, new + lib.path[len(old):]]

    if len(changes):
        subprocess.check_call(['install_name_tool'] + changes + [path])

        # Determine if the library needs to be re-signed.
        signature = subprocess.run(['codesign', '--verify', path],
                                   capture_output=True, text=True)
        if not signature.stderr:
            pass  # The existing signature is valid.
        elif "code object is not signed at all" in signature.stderr:
            pass  # Not signed.
        else:
            subprocess.check_call(['codesign', '--force', '--sign', '-', path])


def main(args):
    # Set up argument parser.
    parser = argparse.ArgumentParser(
        description='Change the prefix(es) of dependent shared libraries.')
    parser.add_argument(
        '--old', required=True, action='append',
        help='Old prefix to be replaced (may be specified multiple times)')
    parser.add_argument(
        '--new', required=True, action='append',
        help='Replacement prefix (may be specified multiple times)')
    parser.add_argument(
        '--wheel',
        help='Modify a file inside the specified wheel')
    parser.add_argument(
        'library', nargs='+',
        help='Dynamic library to modify')

    # Parse arguments.
    options = parser.parse_args(args)
    assert len(options.old) == len(options.new)
    replacements = list(zip(options.old, options.new))

    # Get list of dependent libraries and modify their prefixes.
    if options.wheel:
        with tempfile.TemporaryDirectory() as tmp_dir:
            libs = {}
            for path in options.library:
                lib_path = os.path.join(tmp_dir, f'lib{len(libs)}')
                _extract(options.wheel, path, lib_path)
                libs[path] = lib_path

                _chlpath(lib_path, replacements)

            _replace(options.wheel, libs)

    else:
        for path in options.library:
            _chlpath(path, replacements)


# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if __name__ == '__main__':
    main(sys.argv[1:])
