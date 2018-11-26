# -*- python -*-

load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_package_library",
    "drake_cc_test",
)

def attic_drake_cc_binary(**kwargs):
    """A wrapper to that should be exclusively used within attic/...."""
    drake_cc_binary(
        **kwargs
    )

def attic_drake_cc_googletest(**kwargs):
    """A wrapper to that should be exclusively used within attic/...."""
    drake_cc_googletest(**kwargs)

def attic_drake_cc_library(**kwargs):
    """A wrapper to that should be exclusively used within attic/...."""
    drake_cc_library(
        strip_include_prefix = "/attic",
        **kwargs
    )

def attic_drake_cc_package_library(**kwargs):
    """A wrapper to that should be exclusively used within attic/...."""
    drake_cc_package_library(
        **kwargs
    )

def attic_drake_cc_test(**kwargs):
    """A wrapper to that should be exclusively used within attic/...."""
    drake_cc_test(**kwargs)
