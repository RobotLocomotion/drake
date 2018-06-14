# -*- python -*-

load(
    "//tools/workspace/lcm:lcm.bzl",
    "lcm_cc_library",
    "lcm_java_library",
    "lcm_py_library",
)
load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_installed_headers",
    "installed_headers_for_drake_deps",
)

def drake_lcm_cc_library(
        name,
        deps = [],
        tags = [],
        strip_include_prefix = None,
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""

    # Drake's *.lcm files all live in our //drake/lcmtypes package.  Per LCM
    # upstream convention, the include directory for generated code should
    # always look like "my_lcm_package/my_lcm_struct.h", but by default Bazel
    # would provide "drake/lcmtypes/my_lcm_package/my_lcm_struct.h" as an
    # allowed spelling.  Here, we override that to enforce that Drake's include
    # statements use the standard formulation.  (We allow callers to override
    # our enforcement though, such as for special-case testing.)
    if strip_include_prefix == None:
        strip_include_prefix = "/" + native.package_name()
    detail = lcm_cc_library(
        name = name,
        deps = deps,
        tags = tags + ["nolint"],
        strip_include_prefix = strip_include_prefix,
        **kwargs
    )
    drake_installed_headers(
        name = name + ".installed_headers",
        hdrs = detail.hdrs,
        deps = installed_headers_for_drake_deps(deps),
        tags = ["nolint"],
    )

def drake_lcm_java_library(
        name,
        tags = [],
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
    lcm_java_library(
        name = name,
        tags = tags + ["nolint"],
        **kwargs
    )

def drake_lcm_py_library(
        name,
        tags = [],
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
    lcm_py_library(
        name = name,
        tags = tags + ["nolint"],
        **kwargs
    )
