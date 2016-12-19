# -*- python -*-
# This file contains macros for Bazel; see drake/doc/bazel.rst.

def cc_unit_test(
        name,
        size=None,
        srcs=None,
        deps=None,
        **kwargs):
    """Creates a rule to build a C++ unit test using googletest.  Always adds a
    deps= entry for googletest main.

    By default, sets name="test/${name}.cc" per Drake's filename convention.
    By default, sets size="small" because that indicates a unit test.

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
