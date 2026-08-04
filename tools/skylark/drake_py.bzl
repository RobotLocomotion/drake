load("//tools/flags/internal:multi_config.bzl", "py_test_with_alt_binder")
load(
    "//tools/skylark:kwargs.bzl",
    "amend",
    "combine_conditions",
    "incorporate_allow_network",
    "incorporate_display",
    "incorporate_num_threads",
    "incorporate_rendering",
    "incorporate_test_weight_heuristics",
)
load("//tools/skylark:py.bzl", "py_binary", "py_library", "py_test")

def drake_py_library(
        name,
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
    py_library(
        name = name,
        srcs_version = "PY3",
        **kwargs
    )

def drake_py_binary(
        name,
        srcs = None,
        main = None,
        data = [],
        deps = None,
        tags = [],
        add_test_rule = False,
        test_rule_args = [],
        test_rule_data = [],
        test_rule_tags = None,
        test_rule_size = None,
        test_rule_timeout = None,
        test_rule_flaky = False,
        test_rule_rendering = False,
        test_rule_test_alt_binder = "auto",
        **kwargs):
    """A wrapper to insert Drake-specific customizations.
    """
    if main == None and len(srcs) == 1:
        main = srcs[0]
    py_binary(
        name = name,
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
            # We use the same srcs for both the py_binary and the py_test so we
            # must disable pre-compilation during the py_test target; otherwise
            # both targets would declare an identical set of `*.pyc` output
            # files from their build actions and bazel would error out because
            # of the malformed BUILD file.
            precompile = "disabled",
            args = test_rule_args,
            data = data + test_rule_data,
            size = test_rule_size,
            timeout = test_rule_timeout,
            flaky = test_rule_flaky,
            opt_out_conditions = [
                # Smoke tests don't count as coverage.
                "//tools/kcov:enabled",
            ],
            rendering = test_rule_rendering,
            test_alt_binder = test_rule_test_alt_binder,
            tags = (test_rule_tags or []) + ["nolint"],
            # The added test rule isn't going to `import unittest`, but test
            # dependencies such as numpy(!!) do so unconditionally.  We should
            # allow that.
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

    @param allow_network (optional, default is ["meshcat"])
        See drake/tools/skylark/README.md for details.

    @param num_threads (optional, default is 1)
        See drake/tools/skylark/README.md for details.
    """
    helper = "//common/test_utilities:drake_py_unittest_main.py"
    if kwargs.pop("srcs", None):
        fail("Changing srcs= is not allowed by drake_py_unittest." +
             " Use drake_py_test instead, if you need something weird.")
    srcs = ["test/%s.py" % name, helper]
    drake_py_test(
        name = name,
        srcs = srcs,
        main = helper,
        allow_import_unittest = True,
        _drake_py_unittest_shard_count = kwargs.pop("shard_count", None),
        deps = kwargs.pop("deps", []) + [
            "@xmlrunner_py_internal//:xmlrunner_py",
        ],
        **kwargs
    )

def drake_py_test(
        name,
        size = None,
        srcs = None,
        deps = None,
        allow_import_unittest = False,
        allow_network = None,
        display = False,
        num_threads = None,
        opt_in_condition = None,
        opt_out_conditions = None,
        rendering = False,
        test_alt_binder = "auto",
        **kwargs):
    """A wrapper to insert Drake-specific customizations.

    @param allow_import_unittest (optional, default is False)
        If False, this test (and anything it imports) is prevented from doing
        `import unittest`.  This is a guard against writing `unittest`-based
        cases that accidentally never get run.  In general, `unittest`-based
        tests should use the `drake_py_unittest` macro instead of this one
        (thus disabling this interlock), but can override this parameter in
        case something unique is happening and the other macro can't be used.

    @param allow_network (optional, default is ["meshcat"])
        See drake/tools/skylark/README.md for details.

    @param display (optional, default is False)
        See drake/tools/skylark/README.md for details.

    @param num_threads (optional, default is 1)
        See drake/tools/skylark/README.md for details.

    @param opt_in_condition (optional, default is None)
        See drake/tools/skylark/README.md for details.

    @param opt_out_conditions (optional, default is None)
        See drake/tools/skylark/README.md for details.

    @param rendering (optional, default is False)
        See drake/tools/skylark/README.md for details.

    @param test_alt_binder (optional, default is "auto")
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
    kwargs["shard_count"] = kwargs.pop("_drake_py_unittest_shard_count", None)

    kwargs = incorporate_allow_network(kwargs, allow_network = allow_network)
    kwargs = incorporate_display(kwargs, display = display)
    kwargs = incorporate_num_threads(kwargs, num_threads = num_threads)
    kwargs = incorporate_rendering(kwargs, rendering = rendering)
    kwargs = incorporate_test_weight_heuristics(kwargs)
    kwargs = amend(kwargs, "tags", append = ["py"])
    opt_out_conditions = (opt_out_conditions or []) + kwargs.pop("opt_out_conditions", [])

    deps = deps or []
    if not allow_import_unittest:
        deps = deps + ["//common/test_utilities:disable_python_unittest"]
    target_compatible_with, _ = combine_conditions(
        name = name,
        opt_in_condition = opt_in_condition,
        opt_out_conditions = opt_out_conditions,
    )
    py_test(
        name = name,
        size = size,
        srcs = srcs,
        deps = deps,
        target_compatible_with = target_compatible_with,
        python_version = "PY3",
        srcs_version = "PY3",
        **kwargs
    )
    if test_alt_binder not in (True, False, "auto"):
        fail("test_alt_binder must be set to True, False, or \"auto\"")
    if test_alt_binder == "auto":
        # TODO(#21572) Eventually "auto" should enable relevant tests.
        test_alt_binder = False
    if test_alt_binder:
        alt_target_compatible_with, _ = combine_conditions(
            name = "alt_binder/" + name,
            opt_in_condition = opt_in_condition,
            opt_out_conditions = (opt_out_conditions or []) + [
                # Sanitizers and memcheck use `test_lang_filters` to opt-out of
                # py_tests, but for some reason that filter doesn't work on the
                # alt_binder tests, so we need to skip them explicitly.
                "//tools:using_sanitizer",
                "//tools/valgrind:enabled",
                # Python coverage tests are allowed in `test_lang_filters`, but
                # we actually only want coverage of the primary binder.
                "//tools/kcov:enabled",
            ],
        )
        py_test_with_alt_binder(
            name = "alt_binder/" + name,
            main = kwargs.pop("main", None) or "{}.py".format(name),
            size = size,
            srcs = srcs,
            deps = deps,
            target_compatible_with = alt_target_compatible_with,
            python_version = "PY3",
            srcs_version = "PY3",
            **kwargs
        )

def py_linter_test(
        name,
        **kwargs):
    """Wrapper for py_test, to be used for running a linter."""
    py_test(
        name = name,
        target_compatible_with = select({
            # Skip lint tests in coverage builds.
            "@drake//tools/kcov:enabled": ["@platforms//:incompatible"],
            "//conditions:default": [],
        }),
        **kwargs
    )
