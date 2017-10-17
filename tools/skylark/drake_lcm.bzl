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
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
    lcm_cc_library(
        name = name,
        tags = tags + ["nolint"],
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
