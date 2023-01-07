# -*- python -*-

load("//tools/skylark:py.bzl", "py_binary", "py_library", "py_test")
load("//tools/skylark:kwargs.bzl", "amend", "incorporate_num_threads")

def drake_py_library(
        name,
        deps = None,
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""

    # Work around https://github.com/bazelbuild/bazel/issues/1567.
    deps = (deps or []) + ["//:module_py"]
    py_library(
        name = name,
        deps = deps,
        srcs_version = "PY3",
        **kwargs
    )

def _redirect_test_impl(ctx):
    info = dict(
        bad_target = ctx.attr.bad_target,
        good_target = ctx.attr.good_target,
    )
    content = """#!/bin/bash
echo "ERROR: Please use\n    {good_target}\n  The label\n    {bad_target}\n" \
     " does not exist." >&2
exit 1
""".format(**info)
    ctx.actions.write(
        output = ctx.outputs.executable,
        content = content,
    )
    return [DefaultInfo()]

# Defines a test which will fail when run via `bazel run` or `bazel test`,
# redirecting the user to the correct binary to use. This should typically have
# a "manual" tag.
_redirect_test = rule(
    attrs = {
        "bad_target": attr.string(mandatory = True),
        "good_target": attr.string(mandatory = True),
    },
    test = True,
    implementation = _redirect_test_impl,
)

def _py_target_isolated(
        name,
        py_target = None,
        srcs = None,
        main = None,
        isolate = True,
        visibility = None,
        **kwargs):
    # See #8041 for more details.
    # TODO(eric.cousineau): See if we can remove these shims once we stop
    # supporting Python 2 (#10606).
    if py_target == None:
        fail("Must supply macro function for defining `py_target`.")

    # Targets that are already isolated (with a `py/` prefix) don't require any
    # additional work. This can happen when linting tests (isolated by
    # definition) are invoked for isolated Python targets. Otherwise, they get
    # "doubly isolated" as `py/py/{name}`.
    prefix = "py/"
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
            visibility = visibility,
            **kwargs
        )

        # Disable and redirect original name.
        package_prefix = "//" + native.package_name() + ":"

        # N.B. Make sure that a test (visible to both `bazel run` and
        # `bazel test`) with the original name redirects to the isolated
        # instantiation so users unfamiliar with isolation that use the
        # "obvious" spelling will be properly informed.
        _redirect_test(
            name = name,
            good_target = package_prefix + actual,
            bad_target = package_prefix + name,
            tags = ["manual"],
            visibility = visibility,
        )
    else:
        py_target(
            name = name,
            srcs = srcs,
            main = main,
            visibility = visibility,
            **kwargs
        )

def drake_py_binary(
        name,
        srcs = None,
        main = None,
        data = [],
        deps = None,
        isolate = False,
        tags = [],
        add_test_rule = 0,
        test_rule_args = [],
        test_rule_data = [],
        test_rule_tags = None,
        test_rule_size = None,
        test_rule_timeout = None,
        test_rule_flaky = 0,
        **kwargs):
    """A wrapper to insert Drake-specific customizations.

    @param isolate (optional, default is False)
        If True, the binary will be placed in a folder isolated from the
        library code. This prevents submodules from leaking in as top-level
        submodules. For more detail, see #8041.
    """

    # Work around https://github.com/bazelbuild/bazel/issues/1567.
    deps = deps or []
    if "//:module_py" not in deps:
        deps += ["//:module_py"]
    if main == None and len(srcs) == 1:
        main = srcs[0]
    _py_target_isolated(
        name = name,
        py_target = py_binary,
        isolate = isolate,
        srcs = srcs,
        main = main,
        data = data,
        deps = deps,
        tags = tags,
        python_version = "PY3",
        srcs_version = "PY3",
        **kwargs
    )
    if add_test_rule:
        drake_py_test(
            name = name + "_test",
            srcs = srcs,
            main = main,
            deps = deps,
            isolate = isolate,
            args = test_rule_args,
            data = data + test_rule_data,
            size = test_rule_size,
            timeout = test_rule_timeout,
            flaky = test_rule_flaky,
            tags = (test_rule_tags or []) + ["nolint"],
            # N.B. Same as the warning in `drake_pybind_cc_googletest`: numpy
            # imports unittest unconditionally.
            allow_import_unittest = True,
            **kwargs
        )

def drake_py_unittest(
        name,
        **kwargs):
    """Declares a `unittest`-based python test.

    This macro should be preferred instead of the basic drake_py_test for tests
    that use the `unittest` framework.  Tests that use this macro should *not*
    contain a __main__ handler nor a shebang line.  By default, sets test size
    to "small" to indicate a unit test.

    @param num_threads (optional, default is 1)
        See drake/tools/skylark/README.md for details.
    """
    helper = "//common/test_utilities:drake_py_unittest_main.py"
    if kwargs.pop("srcs", None):
        fail("Changing srcs= is not allowed by drake_py_unittest." +
             " Use drake_py_test instead, if you need something weird.")
    srcs = ["test/%s.py" % name, helper]
    num_threads = kwargs.pop("num_threads", 1)
    kwargs = incorporate_num_threads(kwargs, num_threads = num_threads)
    drake_py_test(
        name = name,
        srcs = srcs,
        main = helper,
        allow_import_unittest = True,
        _drake_py_unittest_shard_count = kwargs.pop("shard_count", None),
        deps = kwargs.pop("deps", []) + [
            "@xmlrunner_py",
        ],
        **kwargs
    )

def drake_py_test(
        name,
        size = None,
        srcs = None,
        deps = None,
        isolate = True,
        allow_import_unittest = False,
        num_threads = None,
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

    @param num_threads (optional, default is 1)
        See drake/tools/skylark/README.md for details.

    By default, sets test size to "small" to indicate a unit test. Adds the tag
    "py" if not already present.

    This macro does not allow a shard_count; use drake_py_unittest for that.
    """
    if size == None:
        size = "small"
    if srcs == None:
        srcs = ["test/%s.py" % name]
    if kwargs.get("shard_count") != None:
        fail("Only drake_py_unittest can use sharding")
    shard_count = kwargs.pop("_drake_py_unittest_shard_count", None)

    kwargs = incorporate_num_threads(kwargs, num_threads = num_threads)
    kwargs = amend(kwargs, "tags", append = ["py"])

    # Work around https://github.com/bazelbuild/bazel/issues/1567.
    deps = deps or []
    if "//:module_py" not in deps:
        deps += ["//:module_py"]
    if not allow_import_unittest:
        deps = deps + ["//common/test_utilities:disable_python_unittest"]
    _py_target_isolated(
        name = name,
        py_target = py_test,
        isolate = isolate,
        size = size,
        shard_count = shard_count,
        srcs = srcs,
        deps = deps,
        python_version = "PY3",
        srcs_version = "PY3",
        **kwargs
    )

def py_test_isolated(
        name,
        **kwargs):
    """Provides a directory-isolated Python test, robust against shadowing
    (#8041).
    """
    _py_target_isolated(
        name = name,
        py_target = py_test,
        isolate = True,
        **kwargs
    )
