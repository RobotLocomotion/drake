# -*- python -*-

load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
)
load(
    "@drake//tools/lint:cpplint.bzl",
    "cpplint_extra",
)

# This file contains macros that help abbreviate patterns that frequently
# appear in the solvers folder.

def _sync_conditions(condition1, condition2):
    """Asserts that one and only one condition is given. Sets the other
    condition to "//conditions:default" and returns them in the same order
    as they were given.
    """
    if int(bool(condition1)) + int(bool(condition2)) != 1:
        fail("Specify exactly one of opt-in or opt-out")
    return (
        condition1 or "//conditions:default",
        condition2 or "//conditions:default",
    )

def drake_cc_variant_library(
        name,
        *,
        opt_in_condition = None,
        opt_out_condition = None,
        srcs_always,
        srcs_enabled,
        srcs_disabled,
        hdrs,
        interface_deps,
        deps_always,
        deps_enabled,
        internal = False,
        visibility = None):
    """Declares a library with a uniform set of header files (typically just
    one header file) but with two different cc file implementations, depending
    on a configuration setting. This is how we turn on/off specific solver
    back-end implementations, e.g., `snopt_solver.cc` vs `no_snopt.cc`.

    The same hdrs are used unconditionally. The interface_deps should list the
    dependencies for the header file(s), and thus are also unconditional.

    Exactly one of opt_in_condition or opt_out_condition must be provided, to
    specify the configuration setting that chooses which cc files to use.

    Open-source solvers are usually ON by default (so, will use opt_out_...).
    Commercial solvers are usually OFF by default (so, will use opt_in_...).

    The sources listed in srcs_always contain definitions that are appropriate
    whether or not a back-end is enabled. This usually contains things such as
    the solver name, ID, and attribute support. The deps_always lists the
    dependencies of these files.

    The sources listed in srcs_enabled contain the code of the fully-featured
    implementation. The deps_enabled lists the dependencies of these srcs.
    The sources listed in srcs_disabled contain the alternative (stub)
    definitions that report failure (e.g., returning false or throwing).

    This rule enables linting of all of the mentioned source files, even if
    the compiler will not build them in the current configuration. We want all
    files to be lint-free, even when the commercial solvers are disabled.
    """
    opt_in_condition, opt_out_condition = _sync_conditions(
        opt_in_condition,
        opt_out_condition,
    )
    drake_cc_library(
        name = name,
        srcs = select({
            opt_in_condition: srcs_always + srcs_enabled,
            opt_out_condition: srcs_always + srcs_disabled,
        }),
        hdrs = hdrs,
        interface_deps = interface_deps,
        deps = select({
            opt_in_condition: deps_always + deps_enabled,
            opt_out_condition: deps_always,
        }),
        internal = internal,
        visibility = visibility,
    )
    cpplint_extra(
        name = name + "_cpplint",
        srcs = hdrs + srcs_always + srcs_enabled + srcs_disabled,
    )

def drake_cc_optional_library(
        name,
        *,
        opt_in_condition = None,
        opt_out_condition = None,
        srcs,
        hdrs,
        copts = None,
        visibility = ["//visibility:private"],
        interface_deps = None,
        deps = None):
    """Declares a private library (package-local, not installed) guarded by a
    configuration setting. When the configuration is disabled, the library is
    totally empty (but still a valid library label). This is used for helper
    or utility code that's called by a fully-feataured back-end implementation.

    Exactly one of opt_in_condition or opt_out_condition must be provided, to
    specify the configuration setting that chooses which cc files to use.

    Open-source solvers are usually ON by default (so, will use opt_out_...).
    Commercial solvers are usually OFF by default (so, will use opt_in_...).

    This rule enables linting of all of the mentioned source files, even if
    the compiler will not build them in the current configuration. We want all
    files to be lint-free, even when the commercial solvers are disabled.
    """
    opt_in_condition, opt_out_condition = _sync_conditions(
        opt_in_condition,
        opt_out_condition,
    )
    drake_cc_library(
        name = name,
        srcs = select({
            opt_in_condition: srcs,
            opt_out_condition: [],
        }),
        hdrs = select({
            opt_in_condition: hdrs,
            opt_out_condition: [],
        }),
        install_hdrs_exclude = select({
            opt_in_condition: hdrs,
            opt_out_condition: [],
        }),
        copts = select({
            opt_in_condition: copts or [],
            opt_out_condition: [],
        }),
        tags = ["exclude_from_package"],
        visibility = visibility,
        interface_deps = None if interface_deps == None else select({
            opt_in_condition: interface_deps,
            opt_out_condition: [],
        }),
        deps = select({
            opt_in_condition: deps or [],
            opt_out_condition: [],
        }),
    )
    cpplint_extra(
        name = name + "_cpplint",
        srcs = hdrs + srcs,
    )

def drake_cc_optional_googletest(
        name,
        *,
        opt_in_condition = None,
        opt_out_condition = None,
        tags = None,
        deps,
        use_default_main = True):
    """Declares a test that is not even compiled under certain configurations.

    This is intended only for testing of drake_cc_optional_library targets,
    where the header file(s) are not even present under certain configurations.
    For testing a drake_cc_variant_library, the header(s) should always be
    available, so there is no reason we avoid compiling the unit test.

    Exactly one of opt_in_condition or opt_out_condition must be provided, to
    specify the configuration setting that chooses which cc files to use.

    Open-source solvers are usually ON by default (so, will use opt_out_...).
    Commercial solvers are usually OFF by default (so, will use opt_in_...).

    This rule enables linting of all of the mentioned source files, even if
    the compiler will not build them in the current configuration. We want all
    files to be lint-free, even when the commercial solvers are disabled.
    """
    opt_in_condition, opt_out_condition = _sync_conditions(
        opt_in_condition,
        opt_out_condition,
    )
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
            opt_out_condition: [],
        }),
        tags = tags,
        deps = select({
            opt_in_condition: deps,
            opt_out_condition: opt_out_deps,
        }),
        use_default_main = use_default_main,
    )
    cpplint_extra(
        name = name + "_cpplint",
        srcs = srcs,
    )
