# -*- python -*-

def drake_py_library(
        name,
        deps = None,
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
    # Work around https://github.com/bazelbuild/bazel/issues/1567.
    deps = (deps or []) + ["//:module_py"]
    native.py_library(
        name = name,
        deps = deps,
        **kwargs)

def _py_target_isolated(
        name,
        py_target = None,
        srcs = None,
        main = None,
        isolate = True,
        visibility = None,
        **kwargs):
    # See #8041 for more details.
    if py_target == None:
        fail("Must supply macro function for defining `py_target`.")
    # Do not isolate targets that are already isolated. This generally happens
    # when linting tests (which are isolated) are invoked for isolated Python
    # targets. Without this check, the actual test turns into
    # `_isolated/_isolated/{name}`.
    prefix = "_isolated/"
    if isolate and not name.startswith(prefix):
        actual = prefix + name
        # Preserve original functionality.
        if not main:
            main = name + ".py"
        if not srcs:
            srcs = [name + ".py"]
        py_target(
            name = actual,
            srcs = srcs,
            main = main,
            visibility = ["//visibility:private"],
            **kwargs)
        native.alias(
            name = name,
            actual = actual,
            visibility = visibility,
        )
    else:
        py_target(
            name = name,
            srcs = srcs,
            main = main,
            visibility = visibility,
            **kwargs)

def drake_py_binary(
        name,
        srcs = None,
        deps = None,
        isolate = False,
        **kwargs):
    """A wrapper to insert Drake-specific customizations.

    @param isolate (optional, default is False)
        If True, the binary will be placed in a folder isolated from the
        library code. This prevents submodules from leaking in as top-level
        submodules. For more detail, see #8041.
    """
    # Work around https://github.com/bazelbuild/bazel/issues/1567.
    deps = (deps or []) + ["//:module_py"]
    _py_target_isolated(
        name = name,
        py_target = native.py_binary,
        isolate = isolate,
        srcs = srcs,
        deps = deps,
        **kwargs)

def drake_py_unittest(
        name,
        srcs = [],
        **kwargs):
    """Declares a `unittest`-based python test.

    This macro should be preferred instead of the basic drake_py_test for tests
    that use the `unittest` framework.  Tests that use this macro should *not*
    contain a __main__ handler nor a shebang line.
    """
    helper = "//common/test_utilities:drake_py_unittest_main.py"
    if not srcs:
        srcs = ["test/%s.py" % name]
    drake_py_test(
        name = name,
        srcs = srcs + [helper],
        main = helper,
        allow_import_unittest = True,
        **kwargs)

def drake_py_test(
        name,
        srcs = None,
        deps = None,
        isolate = True,
        allow_import_unittest = False,
        **kwargs):
    """A wrapper to insert Drake-specific customizations.

    @param isolate (optional, default is True)
        If True, the test binary will be placed in a folder isolated from the
        library code. This prevents submodules from leaking in as top-level
        submodules. For more detail, see #8041.

    @param allow_import_unittest (optional, default is False)
        If False, this test (and anything it imports) is prevented from doing
        `import unittest`.  This is a guard against writing `unittest`-based
        cases that accidentally never get run.  In general, `unittest`-based
        tests should use the `drake_py_unittest` macro instead of this one
        (thus disabling this interlock), but can override this parameter in
        case something unique is happening and the other macro can't be used.
    """
    if srcs == None:
        srcs = ["test/%s.py" % name]
    # Work around https://github.com/bazelbuild/bazel/issues/1567.
    deps = (deps or []) + ["//:module_py"]
    if not allow_import_unittest:
        deps = deps + ["//common/test_utilities:disable_python_unittest"]
    _py_target_isolated(
        name = name,
        py_target = native.py_test,
        isolate = isolate,
        srcs = srcs,
        deps = deps,
        **kwargs)

def py_test_isolated(
        name,
        **kwargs):
    """Provides a directory-isolated Python test, robust against shadowing
    (#8041).
    """
    _py_target_isolated(
        name = name,
        py_target = native.py_test,
        isolate = True,
        **kwargs)
