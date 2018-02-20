# -*- python -*-

load("//tools/skylark:6996.bzl", "adjust_labels_for_drake_hoist")

def drake_py_library(
        name,
        deps = None,
        data = None,
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
    deps = adjust_labels_for_drake_hoist(deps)
    data = adjust_labels_for_drake_hoist(data)
    # Work around https://github.com/bazelbuild/bazel/issues/1567.
    deps = (deps or []) + ["//:module_py"]
    native.py_library(
        name = name,
        deps = deps,
        data = data,
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
        data = None,
        isolate = False,
        **kwargs):
    """A wrapper to insert Drake-specific customizations.

    @param isolate (optional, default is False)
        If True, the binary will be placed in a folder isolated from the
        library code. This prevents submodules from leaking in as top-level
        submodules. For more detail, see #8041.
    """
    deps = adjust_labels_for_drake_hoist(deps)
    data = adjust_labels_for_drake_hoist(data)
    # Work around https://github.com/bazelbuild/bazel/issues/1567.
    deps = (deps or []) + ["//:module_py"]
    _py_target_isolated(
        name = name,
        py_target = native.py_binary,
        isolate = isolate,
        srcs = srcs,
        deps = deps,
        data = data,
        **kwargs)

def drake_py_test(
        name,
        srcs = None,
        deps = None,
        data = None,
        isolate = True,
        **kwargs):
    """A wrapper to insert Drake-specific customizations.

    @param isolate (optional, default is True)
        If True, the test binary will be placed in a folder isolated from the
        library code. This prevents submodules from leaking in as top-level
        submodules. For more detail, see #8041.
    """
    if srcs == None:
        srcs = ["test/%s.py" % name]
    deps = adjust_labels_for_drake_hoist(deps)
    data = adjust_labels_for_drake_hoist(data)
    # Work around https://github.com/bazelbuild/bazel/issues/1567.
    deps = (deps or []) + ["//:module_py"]
    _py_target_isolated(
        name = name,
        py_target = native.py_test,
        isolate = isolate,
        srcs = srcs,
        deps = deps,
        data = data,
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
