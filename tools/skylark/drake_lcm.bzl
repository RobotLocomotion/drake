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
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
    detail = lcm_cc_library(
        name = name,
        deps = deps,
        tags = tags + ["nolint"],
        **kwargs)
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
