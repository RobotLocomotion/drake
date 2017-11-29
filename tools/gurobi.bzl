# -*- python -*-

# This file is a compatibility shim, to ease porting to our new filename.
# TODO(jwnimmer-tri) This is a compatibility shim; remove this file on or
# around 2017-11-01.
print("warning: tools/gurobi.bzl is deprecated; " +
      "use tools/workspace/gurobi/gurobi.bzl.")

load("//tools/workspace/gurobi:gurobi.bzl", "gurobi_test_tags")
