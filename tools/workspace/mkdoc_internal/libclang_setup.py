import os
import platform

from clang import cindex

# Alternative: Make this a function in `mkdoc.py`, and import it from mkdoc as
# a module? (if this was really authored in `mkdoc.py`...)


def add_library_paths(parameters=None):
    """Set library paths for finding libclang on supported platforms.

    Args:
        parameters(list): If not None, it's used for adding parameters which
            are used in `mkdoc.py`.

    Returns:
    """
    # Per install_prereqs, we expect Clang 20 to be installed.
    version = 20
    arch = platform.machine()
    library_file = f"/usr/lib/{arch}-linux-gnu/libclang-{version}.so"
    if not os.path.exists(library_file):
        raise RuntimeError(f"Library file {library_file} does NOT exist")
    cindex.Config.set_library_file(library_file)
