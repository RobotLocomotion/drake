# -*- python -*-

load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_package_library",
    "drake_cc_test",
)

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

def attic_drake_cc_binary(**kwargs):
    """A wrapper to that should be exclusively used within attic/...."""
    drake_cc_binary(
        **kwargs
    )

def attic_drake_cc_googletest(name, *, tags = [], **kwargs):
    """A wrapper to that should be exclusively used within attic/...."""
    new_tags = (tags or []) + _ATTIC_TEST_TAGS
    drake_cc_googletest(
        name,
        tags = new_tags,
        **kwargs
    )

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

def attic_drake_cc_test(name, *, tags = [], **kwargs):
    """A wrapper to that should be exclusively used within attic/...."""
    new_tags = (tags or []) + _ATTIC_TEST_TAGS
    drake_cc_test(
        name,
        tags = new_tags,
        **kwargs
    )
