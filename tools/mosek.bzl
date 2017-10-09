# -*- python -*-

# This file is a compatibility shim, to ease porting to our new filename.
# TODO(jwnimmer-tri) This is a compatibility shim; remove this file on or
# around 2017-11-01.
print("warning: tools/mosek.bzl is deprecated; " +
      "use tools/workspace/mosek/mosek.bzl.")

load("//tools/workspace/mosek:mosek.bzl", "mosek_test_tags")
