# -*- python -*-

# This file is a compatibility shim, to ease porting to our new filename.
# TODO(jwnimmer-tri) This is a compatibility shim; remove this file on or
# around 2017-11-01.
print("warning: tools/lint.bzl is deprecated; use tools/lint/lint.bzl.")

load("//tools/lint:lint.bzl", "add_lint_tests")
