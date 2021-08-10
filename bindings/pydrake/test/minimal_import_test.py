"""Checks that simple 'import pydrake' does not load Drake's C++ native
library.

Note that we can't use Drake's unittest framework for this test case,
because it loads native code as part of its deprecation testing probes.

On macOS, we aren't able to access the psutil.memory_maps(), so we can't
actually run this test. We'll assume that testing on Ubuntu is sufficient.
"""
import sys

if "darwin" in sys.platform:
    sys.exit(0)
import psutil


def _error(message):
    print("error: " + message, file=sys.stderr, flush=True)
    sys.exit(1)


def _is_native_code_loaded():
    """Returns True iff libdrake.so has been loaded yet."""
    this_process = psutil.Process()
    for item in this_process.memory_maps():
        if item.path.endswith("/libdrake.so"):
            return True
    return False


def main():
    # Sanity check: native code is not loaded by the testing framework.
    if _is_native_code_loaded():
        _error("The test framework loaded native code?!")

    # Native code is not loaded after 'import pydrake'.
    import pydrake
    if _is_native_code_loaded():
        _error("An 'import pydrake' native code")

    # Once we import a C++ module, we have native code.
    from pydrake.common import RandomDistribution
    if not _is_native_code_loaded():
        _error("Native code was not loaded?"
               " The _is_native_code_loaded function is probably broken.")


if __name__ == '__main__':
    main()
