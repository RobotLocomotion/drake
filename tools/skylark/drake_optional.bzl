load("@drake//tools/workspace:generate_file.bzl", "generate_file")
load("//tools/lint:cpplint.bzl", "cpplint_extra")
load(
    ":drake_cc.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
)
load(":drake_py.bzl", "drake_py_binary", "drake_py_test")
load(":drake_sh.bzl", "drake_sh_test")
load(":kwargs.bzl", "amend")

# This file contains macros that help abbreviate patterns to make 'optional'
# build targets, meaning that in general, there is some condition (e.g., build
# flag, platform, etc.) under which a target shouldn't even compile, or a
# stubbed implementation should be compiled instead of the fleshed-out one.

def drake_cc_variant_library(
        name,
        *,
        opt_in_condition,
        srcs_always = [],
        srcs_enabled,
        srcs_disabled,
        hdrs,
        deps,
        implementation_deps_always = [],
        implementation_deps_enabled,
        internal = False,
        visibility = None):
    """Declares a library with a uniform set of header files (typically just
    one header file) but with two different cc file implementations, depending
    on a configuration setting. This is how we turn on/off specific solver or
    render-engine back-end implementations.

    The same hdrs are used unconditionally. The deps should list the
    dependencies for the header file(s), and thus are also unconditional.

    The opt_in_condition specifies the configuration setting that chooses
    which cc files to use.

    The sources listed in srcs_always contain definitions that are appropriate
    whether or not a back-end is enabled, if such definitions are necessary.
    The implementation_deps_always lists the dependencies of these files.

    The sources listed in srcs_enabled contain the code of the fully-featured
    implementation. The implementation_deps_enabled lists the dependencies of
    these files.

    The sources listed in srcs_disabled contain the alternative (stub)
    definitions that report failure (e.g., returning false or throwing).

    This rule enables linting of all of the mentioned source files, even if
    the compiler will not build them in the current configuration. We want all
    files to be lint-free, even when the commercial solvers are disabled.

    Additionally, declares a library {name}_disabled with the disabled flavor,
    intended for use by unit test checks of stub implementation even when the
    full implementation is enabled.
    """
    drake_cc_library(
        name = name,
        srcs = select({
            opt_in_condition: srcs_always + srcs_enabled,
            "//conditions:default": srcs_always + srcs_disabled,
        }),
        hdrs = hdrs,
        deps = deps,
        implementation_deps = select({
            opt_in_condition: (
                implementation_deps_always + implementation_deps_enabled
            ),
            "//conditions:default": implementation_deps_always,
        }),
        internal = internal,
        visibility = visibility,
    )
    drake_cc_library(
        name = name + "_disabled",
        srcs = srcs_always + srcs_disabled,
        hdrs = hdrs,
        deps = deps,
        implementation_deps = implementation_deps_always,
        testonly = True,
        tags = ["manual"],
        visibility = ["//visibility:private"],
    )
    cpplint_extra(
        name = name + "_cpplint",
        srcs = hdrs + srcs_always + srcs_enabled + srcs_disabled,
    )

def drake_cc_optional_library(
        name,
        *,
        opt_in_condition,
        srcs = [],
        hdrs,
        copts = None,
        internal = False,
        testonly = False,
        visibility = ["//visibility:private"],
        deps = None,
        implementation_deps = None):
    """Declares a private library (package-local, not installed) guarded by a
    configuration setting. When the configuration is disabled, the library is
    totally empty (but still a valid library label). This is used for helper
    or utility code that's called by a fully-feataured back-end implementation.

    The opt_in_condition specifies the configuration setting that chooses
    which cc files to use.

    This rule enables linting of all of the mentioned source files, even if
    the compiler will not build them in the current configuration.
    """
    drake_cc_library(
        name = name,
        srcs = select({
            opt_in_condition: srcs,
            "//conditions:default": [],
        }),
        hdrs = select({
            opt_in_condition: hdrs,
            "//conditions:default": [],
        }),
        # N.B. 'internal' (passed below) excludes the headers already.
        install_hdrs_exclude = [] if internal else select({
            opt_in_condition: hdrs,
            "//conditions:default": [],
        }),
        # N.B. 'internal' (passed below) adds this tag already.
        tags = [] if internal else ["exclude_from_package"],
        copts = select({
            opt_in_condition: copts or [],
            "//conditions:default": [],
        }),
        deps = select({
            opt_in_condition: deps or [],
            "//conditions:default": [],
        }),
        implementation_deps = None if implementation_deps == None else select({
            opt_in_condition: implementation_deps,
            "//conditions:default": [],
        }),
        testonly = testonly,
        internal = internal,
        visibility = visibility,
    )
    cpplint_extra(
        name = name + "_cpplint",
        srcs = hdrs + srcs,
    )

def drake_cc_optional_binary(
        name,
        *,
        opt_in_condition,
        srcs,
        copts = None,
        testonly = False,
        visibility = ["//visibility:private"],
        deps = None,
        data = None):
    """Declares a private binary (package-local, not installed) guarded by a
    configuration setting. When the configuration is disabled, the binary is
    totally empty (but still a valid binary label). This is used for helper
    or utility code that's called by a fully-feataured back-end implementation.

    The opt_in_condition specifies the configuration setting that chooses
    which cc files to use.

    This rule enables linting of all of the mentioned source files, even if
    the compiler will not build them in the current configuration.
    """

    # Always generate a dummy file with an empty main() function so that the
    # target can link when disabled (i.e., it has no srcs).
    empty_main_file_name = name + "_empty_main.cc"
    generate_file(
        name = empty_main_file_name,
        content = "int main() { return 0; }",
    )
    drake_cc_binary(
        name = name,
        srcs = select({
            opt_in_condition: srcs,
            "//conditions:default": [empty_main_file_name],
        }),
        copts = select({
            opt_in_condition: copts or [],
            "//conditions:default": [],
        }),
        deps = deps,  # TODO
        testonly = testonly,
        visibility = visibility,
    )
    cpplint_extra(
        name = name + "_cpplint",
        srcs = srcs,
    )

def drake_py_optional_binary(
        name,
        *,
        opt_in_condition,
        # srcs = None, # TODO
        data = [],
        deps = None,
        **kwargs):
    """Declares a private binary (package-local, not installed) guarded by a
    configuration setting. When the configuration is disabled, the binary is
    totally empty (but still a valid binary label). This is used for helper
    or utility code that's called by a fully-feataured back-end implementation.

    The opt_in_condition specifies the configuration setting that chooses
    which srcs to use.
    """
    drake_py_binary(
        name = name,
        data = select({
            opt_in_condition: data or [],
            "//conditions:default": [],
        }),
        deps = select({
            opt_in_condition: deps,
            "//conditions:default": [],
        }),
        **kwargs
    )

def drake_cc_optional_googletest(
        name,
        *,
        opt_in_condition,
        tags = None,
        deps,
        use_default_main = True,
        **kwargs):
    """Declares a test that is not even compiled under certain configurations.

    This is intended only for testing of drake_cc_optional_library targets,
    where the header file(s) are not even present under certain configurations.
    For testing a drake_cc_variant_library, the header(s) should always be
    available, so there is no reason we avoid compiling the unit test.

    The opt_in_condition specifies the configuration setting that chooses
    which cc files to use.

    This rule enables linting of all of the mentioned source files, even if
    the compiler will not build them in the current configuration.
    """
    srcs = ["test/{}.cc".format(name)]
    if use_default_main:
        opt_out_deps = []
    else:
        # The `srcs` provide a main() function, but in the opt_out case we
        # won't be linking them; to cope, we'll fall back to the default main.
        opt_out_deps = ["//common/test_utilities:drake_cc_googletest_main"]
    drake_cc_googletest(
        name = name,
        srcs = select({
            opt_in_condition: srcs,
            "//conditions:default": [],
        }),
        tags = tags,
        deps = select({
            opt_in_condition: deps,
            "//conditions:default": opt_out_deps,
        }),
        use_default_main = use_default_main,
        **kwargs
    )
    cpplint_extra(
        name = name + "_cpplint",
        srcs = srcs,
    )

def drake_py_optional_unittest(
        name,
        *,
        opt_in_condition,
        **kwargs):
    """Declares a test that trivially passes under certain configurations.

    This is intended only for testing of Python targets created by
    drake_py_optional_binary.

    The opt_in_condition specifies the configuration setting that chooses
    whether to use the srcs specified.
    """
    helper = "//common/test_utilities:drake_py_unittest_main.py"
    if kwargs.pop("srcs", None):
        fail("Changing srcs= is not allowed by drake_py_optional_unittest." +
             " Use drake_py_test instead, if you need something weird.")
    srcs = ["test/%s.py" % name, helper]
    # Generate an empty file to use as a test for each src, for when
    # opt_in_condition is False.
    noop_srcs = []
    for i, src in enumerate(srcs):
        if src != helper:
            noop_src = srcs[i].replace("test/", "test/empty-")
            generate_file(
                name = noop_src,
                content = "",
            )
            noop_srcs.append(noop_src)
    noop_srcs.append(helper)

    # kcov is only appropriate for small-sized unit tests. If a test needs a
    # shard_count or a special timeout, we assume it is not small.
    if "shard_count" in kwargs or "timeout" in kwargs:
        amend(kwargs, "tags", append = ["no_kcov"])

    drake_py_test(
        name = name,
        srcs = select({
            opt_in_condition: srcs,
            "//conditions:default": noop_srcs,
        }),
        main = helper,
        _drake_py_unittest_shard_count = kwargs.pop("shard_count", None),
        deps = select({
            opt_in_condition: kwargs.pop("deps", []) + [
                "@xmlrunner_py_internal//:xmlrunner_py",
            ],
            "//conditions:default": [],
        }),
        data = select({
            opt_in_condition: kwargs.pop("data", []),
            "//conditions:default": [],
        }),
        **kwargs
    )

def drake_sh_optional_test(
        name,
        *,
        opt_in_condition,
        srcs,
        data,
        **kwargs):
    """Declares a test that trivially passes under certain configurations.

    This is intended only for additional testing of drake_cc_optional_googletest
    targets, where a test executable is built with a trivial 'main' function
    under certain configurations. As such, the configurations under which this
    test is applied should match those under which the
    drake_cc_optional_googletest is compiled.

    The opt_in_condition specifies the configuration setting that chooses
    whether to use the srcs specified.
    """

    # Generate a dummy script to use for 'srcs' in the opt-out case.
    empty_sh_name = name + "_empty.sh"
    generate_file(
        name = name + "_empty.sh",
        content = "",
    )
    drake_sh_test(
        name = name,
        srcs = select({
            opt_in_condition: srcs,
            "//conditions:default": [empty_sh_name],
        }),
        data = select({
            opt_in_condition: data,
            "//conditions:default": [],
        }),
        **kwargs
    )

def drake_genrule_optional(
        name,
        *,
        opt_in_condition,
        srcs,
        outs,
        cmd,
        visibility = ["//visibility:private"]):
    """Declares a genrule that doesn't run under certain configurations.

    This is intended for genrules corresponding with build targets created by
    the other macros in this file, for which it isn't necessary to generate the
    output under the configurations where such targets aren't compiled or run.

    The opt_in_condition specifies the configuration setting that chooses
    whether to use the cmd specified to generate the otus from the srcs.
    """
    native.genrule(
        name = name,
        srcs = select({
            opt_in_condition: srcs,
            "//conditions:default": [],
        }),
        outs = outs,
        cmd = select({
            opt_in_condition: cmd,
            # Create empty outputs, since the 'outs' of a genrule can't be
            # conditional.
            "//conditions:default": "touch $(OUTS)",
        }),
        visibility = visibility,
    )
