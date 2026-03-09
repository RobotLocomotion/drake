#!/usr/bin/env python

from pathlib import Path
import sys
from zipfile import ZipFile

# Everything here must have its license installed as part of the wheel build.
# This list must be kept in sync with the calls to copy_ubuntu_license in
# drake/tools/wheel/image/build-wheel.sh.
_ALLOWED_LIBS = frozenset({
    "libgfortran",
    "libgomp",
    "libquadmath",
})

# TODO(jwnimmer-tri) Install the license texts for these (or stop using the
# library entirely).
_EXTRA_LIBS_MACOS = frozenset({
    "libgcc_s",
    "libglib",
    "libintl",
    "libpcre2",
})

if sys.platform == "darwin":
    _ALLOWED_LIBS_ALL = frozenset(_ALLOWED_LIBS | _EXTRA_LIBS_MACOS)
else:
    _ALLOWED_LIBS_ALL = _ALLOWED_LIBS

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

    # Only allow expected shared libraries to be shipped.
    if any([str(path).startswith("drake.libs/"),
            str(path).startswith("drake/.dylibs/")]):
        stem = path.name.split(".")[0]  # Remove filename extension(s).
        stem = stem.split("-")[0]       # Remove anything after the '-'.
        return stem in _ALLOWED_LIBS_ALL

    # Branch on the top-level directory name.
    top_dir = str(path.parents[-2])
    if top_dir.endswith(".dist-info"):
        # Wheel metadata is allowed.
        return True
    elif top_dir in ("drake", "pydrake"):
        # All of our actual code and data is allowed.
        return True
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


assert __name__ == "__main__"
check_files(wheel_path=Path(sys.argv[1]))
