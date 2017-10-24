
This directory contains build system files related to the Bazel build system.
  https://bazel.build/

This folder is intended to house code to support Drake's shared library(ies)
that are part of Drake's `//:install` target for packaging a binary release.

`drake.cps` is the Common Package Specification for Drake, that provides the
necessary information for Drake to be consumed by other projects. Right now it
is hand edited. This is used to generate `drake-config.cmake` via `cps2cmake`.
  https://mwoehlke.github.io/cps/
  https://github.com/mwoehlke/pycps/
