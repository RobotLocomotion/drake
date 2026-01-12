load("//tools/lint:cpplint.bzl", "cpplint_extra")
load(":drake_cc.bzl", "drake_cc_googletest", "drake_cc_library")

# This file contains macros that help abbreviate patterns to make 'optional'
# build targets, meaning that in general, there is some condition (e.g., build
# flag, platform, etc.) under which a target shouldn't even compile, or a
# stubbed implementation should be compiled instead of the fleshed-out one.

def drake_cc_variant_library(
        name,
        *,
        opt_in_condition,
        srcs_always,
        srcs_enabled,
        srcs_disabled,
        hdrs,
        deps,
        implementation_deps_always,
        implementation_deps_enabled,
        internal = False,
        visibility = None):
    """Declares a library with a uniform set of header files (typically just
    one header file) but with two different cc file implementations, depending
    on a configuration setting. This is how we turn on/off specific components
    of our C++ implementation at build-time.

    The same hdrs are used unconditionally. The deps should list the
    dependencies for the header file(s), and thus are also unconditional.

    The opt_in_condition specifies the configuration setting that chooses
    which cc files to use.

    The sources listed in srcs_always contain definitions that are appropriate
    whether or not a feature is enabled, if such definitions are necessary.
    The implementation_deps_always lists the dependencies of these files.

    The sources listed in srcs_enabled contain the code of the fully-featured
    implementation. The implementation_deps_enabled lists the dependencies of
    these files.

    The sources listed in srcs_disabled contain the alternative (stub)
    definitions that report failure (e.g., returning false or throwing).

    This rule enables linting of all of the mentioned source files, even if
    the compiler will not build them in the current configuration. We want all
    files to be lint-free, even when the feature is disabled.

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
        srcs,
        hdrs,
        copts = None,
        defines = None,
        visibility = ["//visibility:private"],
        deps = None,
        implementation_deps = None):
    """Declares a private library (package-local, not installed) guarded by a
    configuration setting. When the configuration is disabled, the library is
    totally empty (but still a valid library label). This is used for helper
    or utility code that's called by a fully-featured implementation.

    The opt_in_condition specifies the configuration setting that chooses
    which cc files to use.

    This rule enables linting of all of the mentioned source files, even if
    the compiler will not build them in the current configuration. We want all
    files to be lint-free, even when the feature is disabled.
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
        install_hdrs_exclude = select({
            opt_in_condition: hdrs,
            "//conditions:default": [],
        }),
        defines = defines,
        copts = select({
            opt_in_condition: copts or [],
            "//conditions:default": [],
        }),
        tags = ["exclude_from_package"],
        visibility = visibility,
        deps = select({
            opt_in_condition: deps or [],
            "//conditions:default": [],
        }),
        implementation_deps = None if implementation_deps == None else select({
            opt_in_condition: implementation_deps,
            "//conditions:default": [],
        }),
    )
    cpplint_extra(
        name = name + "_cpplint",
        srcs = hdrs + srcs,
    )

def drake_cc_optional_googletest(
        name,
        *,
        opt_in_condition,
        timeout = None,
        tags = [],
        data = [],
        deps,
        display = False,
        use_default_main = True):
    """Declares a test that is not even compiled under certain configurations.

    This is intended only for testing of drake_cc_optional_library targets,
    where the header file(s) are not even present under certain configurations.
    For testing a drake_cc_variant_library, the header(s) should always be
    available, so there is no reason we avoid compiling the unit test.

    The opt_in_condition specifies the configuration setting that chooses
    which cc files to use.

    This rule enables linting of all of the mentioned source files, even if
    the compiler will not build them in the current configuration. We want all
    files to be lint-free, even when the feature is disabled.
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
        timeout = timeout,
        tags = tags,
        data = select({
            opt_in_condition: data,
            "//conditions:default": [],
        }),
        deps = select({
            opt_in_condition: deps,
            "//conditions:default": opt_out_deps,
        }),
        display = display,
        use_default_main = use_default_main,
    )
    cpplint_extra(
        name = name + "_cpplint",
        srcs = srcs,
    )
