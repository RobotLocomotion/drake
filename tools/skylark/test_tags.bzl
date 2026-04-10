# These macros are intended to be used when declaring tests that either may-use
# or must-use dependencies that have constraints (commercial licenses, or
# peculiar behavior). For commercial dependnecies, these labels both account
# for any license-related needs and provide a marker so that //tools/bazel.rc
# can selectively enable tests based on the developer's chosen configuration.

def vtk_test_tags():
    """Returns test tags necessary for rendering tests. (This is called "vtk"
    tags, but is relevant even for rendering tests that don't use VTK.)
    """
    return [
        # Disable under LeakSanitizer and Valgrind Memcheck due to
        # driver-related leaks. For more information, see #7520.
        "no_lsan",
        "no_memcheck",
        # Similar to #7520, the GL vendor's libraries are not sufficiently
        # instrumented for compatibility with TSan.
        "no_tsan",
        # Mitigates driver-related issues when running under `bazel test`. For
        # more information, see #7004.
        "no-sandbox",
    ]
