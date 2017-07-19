# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

def pick_and_place_test(args = None, size = "small", srcs = None, **kwargs):
    if srcs == None:
        srcs = ["monolithic_pick_and_place_demo"]

    # The pick and place tests take 10-20 seconds to run normally, but
    # 5-10 minutes to run in debug mode.  Short circuit most of the
    # test in the debug case.
    args += select({"//tools:debug": ["--quick"], "//conditions:default": []})
    native.sh_test(args = args, size = size, srcs = srcs, **kwargs)
