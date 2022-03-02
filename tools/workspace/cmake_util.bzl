# -*- python -*-

# This file contains some helper functions that are meant to be used with
# CMake imported targets.

def split_cmake_list(cmake_list_str):
    """Convert a string containing a CMake-style list into a 'proper' list."""
    if len(cmake_list_str) == 0:
        return []
    return cmake_list_str.split(";")

def _is_library_extension(ext):
    """Return True if ext looks like a library extension."""
    if ext in ["a", "so", "dylib"]:
        return True
    if ext.startswith("so."):
        return True
    if ext.endswith(".dylib"):
        return True

    return False

def library_to_linkopts(path):
    """Convert absolute path to a library to suitable linkopts."""
    opts = []

    if not path.startswith("/"):
        fail("{} is not an absolute path.".format(path))

    dirname, libname = path.rsplit("/", 1)

    # Add `-Wl,-rpath,<path>` for `-L<path>`.
    # See https://github.com/RobotLocomotion/drake/issues/7387#issuecomment-359952616  # noqa
    opts.append("-Wl,-rpath," + dirname)
    opts.append("-L" + dirname)

    if "." in libname:
        ext = libname.split(".", 1)[1]
    else:
        ext = ""

    if not _is_library_extension(ext):
        fail("{} does not appear to be a path to a library.".format(path))

    if not libname.startswith("lib"):
        fail("Name of library {} must start with `lib`.".format(libname))

    opts.append("-l" + libname[3:].split(".", 1)[0])

    return opts
