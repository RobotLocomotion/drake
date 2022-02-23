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

def library_to_linkopts(path):
    """Convert absolute path to a library to suitable linkopts."""
    if "." in path:
        ext = path.split(".", 1)[1]
    else:
        ext = ""

    if ext == ".a" or ext == ".so" or ext.startswith(".so.") or ext.endswith(".dylib"):
        fail("{} does not appear to be a path to a library.".format(path))

    opts = []

    if "/" in path:
        dirname, libname = path.rsplit("/", 1)

        if not dirname in _system_libdirs():
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
