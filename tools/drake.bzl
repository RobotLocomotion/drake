# -*- python -*-

# This file contains macros for the Bazel build system.
# See http://bazel.io/ .

def _component_srcs(names):
    """Given a list of component names, returns the matching list of srcs."""
    return [name + ".cc" for name in names]

def _component_hdrs(names):
    """Given a list of component names, returns the matching list of hdrs.
    The result contains the related -inl.h file iff such a file exists."""
    result = [name + ".h" for name in names]
    for name in names:
        result.extend(native.glob([name + "-inl.h"]))
    return result

def drake_component(
        name,
        srcs=None,
        hdrs=None,
        linkstatic=None,
        visibility=None,
        **kwargs):
    """Creates a rule to build a single C++ component (one header file, its
    associated cc file, and the -inl.h iff such a file exists).

    The default srcs and hdrs can be overridden by supplying an alternative
    list in the srcs= and/or hdrs= args; any user-supplied value replaces the
    default for that arg.  (The most comment use for this is marking header-
    only components as srcs=[].)

    By default, the component is marked static and public.

    """
    if srcs == None: srcs = _component_srcs([name])
    if hdrs == None: hdrs = _component_hdrs([name])
    if linkstatic == None: linkstatic = 1
    if visibility == None: visibility = ["//visibility:public"]
    native.cc_library(
        name=name,
        srcs=srcs,
        hdrs=hdrs,
        linkstatic=linkstatic,
        visibility=visibility,
        **kwargs)

def drake_library(
        name,
        components=None,
        srcs=None,
        hdrs=None,
        linkstatic=None,
        visibility=None,
        **kwargs):
    """Creates a rule to build a C++ library, given a list of component names.

    A C++ component is one header file, its associated cc file, and the -inl.h
    iff such a file exists.  Extra sources or headers that are not components
    may be additionally supplied as srcs= and/or hdrs=.

    By default, the library is marked static and public.

    """
    if len(components or []) > 0:
         if srcs == None: srcs = []
         if hdrs == None: hdrs = []
         srcs += _component_srcs(components)
         hdrs += _component_hdrs(components)
    if linkstatic == None: linkstatic = 1
    if visibility == None: visibility = ["//visibility:public"]
    native.cc_library(
        name=name,
        srcs=srcs,
        hdrs=hdrs,
        linkstatic=linkstatic,
        visibility=visibility,
        **kwargs)
