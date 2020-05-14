# -*- python -*-

load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_package_library",
    "drake_cc_test",
)

_ATTIC_COPTS = [
    # Everything in these packages is deprecated.
    "-Wno-deprecated-declarations",
]

# The attic code is deprecated and is not receiving many (if any) updates, so
# the chance of breaking these variants is small.  We'll conserve CI resources
# by skipping them.
_ATTIC_TEST_TAGS = [
    "no_asan",
    "no_drd",
    "no_helgrind",
    "no_valgrind_tools",
    "no_kcov",
    "no_lsan",
    "no_memcheck",
    "no_tsan",
    "no_ubsan",
]

_ATTIC_DEPS = [
    # Everything in these packages requires a deprecation warning.
    "//attic:attic_warning",
]

def attic_drake_cc_binary(name, *, copts = [], deps = [], **kwargs):
    """A wrapper to that should be exclusively used within attic/...."""
    new_copts = (copts or []) + _ATTIC_COPTS
    new_deps = (deps or []) + _ATTIC_DEPS
    drake_cc_binary(
        name,
        copts = new_copts,
        deps = new_deps,
        **kwargs
    )

def attic_drake_cc_googletest(
        name,
        *,
        copts = [],
        tags = [],
        deps = [],
        **kwargs):
    """A wrapper to that should be exclusively used within attic/...."""
    new_copts = (copts or []) + _ATTIC_COPTS
    new_tags = (tags or []) + _ATTIC_TEST_TAGS
    new_deps = (deps or []) + _ATTIC_DEPS
    drake_cc_googletest(
        name,
        copts = new_copts,
        tags = new_tags,
        deps = new_deps,
        **kwargs
    )

def attic_drake_cc_library(name, *, copts = [], deps = [], **kwargs):
    """A wrapper to that should be exclusively used within attic/...."""
    new_copts = (copts or []) + _ATTIC_COPTS
    new_deps = (deps or []) + _ATTIC_DEPS
    drake_cc_library(
        name,
        strip_include_prefix = "/attic",
        copts = new_copts,
        deps = new_deps,
        **kwargs
    )

def attic_drake_cc_package_library(**kwargs):
    """A wrapper to that should be exclusively used within attic/...."""
    drake_cc_package_library(
        **kwargs
    )

def attic_drake_cc_test(name, *, copts = [], tags = [], deps = [], **kwargs):
    """A wrapper to that should be exclusively used within attic/...."""
    new_copts = (copts or []) + _ATTIC_COPTS
    new_tags = (tags or []) + _ATTIC_TEST_TAGS
    new_deps = (deps or []) + _ATTIC_DEPS
    drake_cc_test(
        name,
        copts = new_copts,
        tags = new_tags,
        deps = new_deps,
        **kwargs
    )
