# -*- python -*-

def drake_cc_library(
        name,
        hdrs=None,
        srcs=None,
        deps=None,
        **kwargs):
    """Creates a rule to declare a C++ library."""
    native.cc_library(
        name=name,
        hdrs=hdrs,
        srcs=srcs,
        deps=deps,
        **kwargs)

def drake_cc_binary(
        name,
        hdrs=None,
        srcs=None,
        deps=None,
        **kwargs):
    """Creates a rule to declare a C++ binary."""
    native.cc_binary(
        name=name,
        hdrs=hdrs,
        srcs=srcs,
        deps=deps,
        **kwargs)

def cc_googletest(
        name,
        size=None,
        srcs=None,
        deps=None,
        **kwargs):
    """Creates a rule to declare a C++ unit test using googletest.  Always adds a
    deps= entry for googletest main (@gtest//:main).

    By default, sets size="small" because that indicates a unit test.
    By default, sets name="test/${name}.cc" per Drake's filename convention.

    """
    if size == None:
        size = "small"
    if srcs == None:
        srcs = ["test/%s.cc" % name]
    if deps == None:
        deps = []
    deps.append("@gtest//:main")
    native.cc_test(
        name=name,
        size=size,
        srcs=srcs,
        deps=deps,
        **kwargs)
