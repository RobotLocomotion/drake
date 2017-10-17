
This directory contains build system files related to the Bazel build system.
  https://bazel.build/

Specifically, files named `*.bzl` are Skylark extensions.
  https://bazel.build/versions/master/docs/skylark/concepts.html

This folder is intended to house skylark code used by Drake.  It contains both
generic functions such as `pathutils.bzl`, and widely-used but drake-specific
functions such as the `drake_{cc,java,py,...}.bzl` language-specific helpers.
