# -*- python -*-

load("@drake_detected_os//:os.bzl", "DISTRIBUTION")

def _system_libdirs():
    """Return the list of 'system' library paths, i.e. those that shouldn't be
    turned into -L arguments."""
    if DISTRIBUTION == "ubuntu":
        return [
            "/usr/lib",
            "/usr/lib/x86_64-linux-gnu",
        ]
    elif DISTRIBUTION == "macos":
        return [
            "/usr/lib",
            "/usr/local/lib",
        ]
    else:
        fail("Operating system {} is NOT supported.".format(DISTRIBUTION))

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
    if "." in path:
        ext = path.split(".", 1)[1]
    else:
        ext = ""

    if not _is_library_extension(ext):
        fail("{} does not appear to be a path to a library.".format(path))

    opts = []

    if "/" in path:
        dirname, libname = path.rsplit("/", 1)

        if dirname not in _system_libdirs():
            # Add `-Wl,-rpath,<path>` for `-L<path>`.
            # See https://github.com/RobotLocomotion/drake/issues/7387#issuecomment-359952616  # noqa
            opts.append("-Wl,-rpath," + dirname)
            opts.append("-L" + dirname)

    else:
        libname = path

    if not libname.startswith("lib"):
        fail("Name of library {} must start with `lib`.".format(libname))

    opts.append("-l" + libname[3:].split(".", 1)[0])

    return opts
