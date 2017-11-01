# -*- python -*-

load(
    "//tools/workspace/lcm:lcm.bzl",
    "lcm_cc_library",
    "lcm_java_library",
    "lcm_py_library",
)

def drake_lcm_cc_library(
        name,
        tags = [],
        strip_include_prefix = None,
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
    # Drake's *.lcm files all live in our //drake/lcmtypes package.  However,
    # per LCM upstream convention, the include directory for generated code
    # should always look like "my_lcm_package/my_lcm_struct.h".  By default,
    # Bazel would provide "drake/lcmtypes/my_lcm_package/my_lcm_struct.h" as
    # the desired spelling.  Here, we override that to force Drake's include
    # statements to use the standard formulation.
    if not strip_include_prefix:
        strip_include_prefix = "/" + native.package_name()
    lcm_cc_library(
        name = name,
        tags = tags + ["nolint"],
        strip_include_prefix = strip_include_prefix,
        **kwargs)

def drake_lcm_java_library(
        name,
        tags = [],
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
    lcm_java_library(
        name = name,
        tags = tags + ["nolint"],
        **kwargs)

def drake_lcm_py_library(
        name,
        tags = [],
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
    lcm_py_library(
        name = name,
        tags = tags + ["nolint"],
        **kwargs)
