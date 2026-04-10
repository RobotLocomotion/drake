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
