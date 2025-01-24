#!/usr/bin/env python

from pathlib import Path
import re
import sys
from zipfile import ZipFile


# Everything here must have its license installed as part of the wheel build.
# This list must be kept in sync with the calls to copy_ubuntu_license in
# drake/tools/wheel/image/build-wheel.sh.
_ALLOWED_LIBS = {
    "libblas",
    "libgfortran",
    "libgomp",
    "liblapack",
    "libquadmath",
    # These are installed by a Drake's repository rule, so don't need copyright
    # installed in build-wheel.sh.
    "libmosek64",
    "libtbb",
}


def get_all_files_in_wheel(wheel_path: Path):
    """Returns and iterator over all (relative) paths the wheel."""
    with ZipFile(wheel_path) as whl:
        for info in whl.infolist():
            if info.is_dir():
                continue
            yield Path(info.filename)


def is_good_file(path):
    """Returns True if the file should be allowed in our wheel."""
    # Check for top-level files.
    if len(path.parents) == 1:
        if path.name.startswith("drake.cpython"):
            # Allow the weird little "drake.cpython....so" stub file.
            return True
        # Don't allow any other top-level files.
        return False

    # Branch on the top-level directory name.
    top_dir = str(path.parents[-2])
    if top_dir.endswith(".dist-info"):
        # Wheel metadata is allowed.
        return True
    elif top_dir in ("drake", "pydrake"):
        # All of our actual code and data is allowed.
        return True
    elif top_dir == "drake.libs":
        # Only allow expected shared libraries to be shipped.
        stem = path.name.split("-")[0]
        return stem in _ALLOWED_LIBS
    else:
        return False


def check_files(wheel_path: Path):
    """Reports whether any bad files are in the wheel."""
    bad_files = [
        path
        for path in get_all_files_in_wheel(wheel_path)
        if not is_good_file(path)
    ]
    if not bad_files:
        return
    print("ERROR: The following paths are not expected in the wheel:")
    for path in bad_files:
        print(f"  {path}")
    sys.exit(1)


# Only run this test on Linux (for now?).
if 'darwin' not in sys.platform:
    check_files(wheel_path=Path(sys.argv[1]))
