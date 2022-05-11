# -*- python -*-

load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_cc_library",
)
load(
    "@drake//tools/lint:cpplint.bzl",
    "cpplint_extra",
)

# This file contains macros that help abbreviate patterns that frequently
# appear in the solvers folder.

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
        deps_enabled):
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
    if int(bool(opt_in_condition)) + int(bool(opt_out_condition)) != 1:
        fail("Choose exactly one of opt-in or opt-out")
    if opt_out_condition:
        opt_in_condition = "//conditions:default"
    else:
        opt_out_condition = "//conditions:default"
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
    """Declares a private library  (package-local, not installed) guarded by a
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
    if int(bool(opt_in_condition)) + int(bool(opt_out_condition)) != 1:
        fail("Choose exactly one of opt-in or opt-out")
    if opt_out_condition:
        opt_in_condition = "//conditions:default"
    else:
        opt_out_condition = "//conditions:default"
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
