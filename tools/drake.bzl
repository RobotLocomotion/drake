# -*- python -*-
# This file contains macros for Bazel; see drake/doc/bazel.rst.

def cc_component(
        name,
        srcs=None,
        hdrs=None,
        linkstatic=None,
        **kwargs):
    """Creates a rule to build a single C++ component (one header file, its
    associated cc file, and the -inl.h iff such a file exists).

    The default srcs and hdrs can be overridden by supplying an alternative
    list in the srcs= and/or hdrs= args; any user-supplied value replaces the
    default for that arg.  (The most comment use for this is marking header-
    only components as srcs=[].)

    By default, the component is marked linkstatic=1.

    """
    if srcs == None:
        srcs = [name + ".cc"]
    if hdrs == None:
        hdrs = [name + ".h"]
        hdrs.extend(native.glob([name + "-inl.h"]))
    if linkstatic == None:
        linkstatic = 1
    native.cc_library(
        name=name,
        srcs=srcs,
        hdrs=hdrs,
        linkstatic=linkstatic,
        **kwargs)

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
