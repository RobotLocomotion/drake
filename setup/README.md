# Installation Prerequisites

These are dependencies that are not pulled in via Bazel and must be installed
on the OS itself.

For how to install Drake on your system, please see the
[Installation and Quickstart](https://drake.mit.edu/installation.html)
documentation.

If you are a developer wishing to add dependencies, please see the
[Jenkins pages for Updating Installation Prerequisites](
https://drake.mit.edu/jenkins.html#updating-installation-prerequisites).

## Layout

The files at `drake/setup/*/binary_distribution/*` are coped into our binary
distribution packages (`drake-NNNN.tar.gz`).

XXX The order of precedence (stacking) is:
- binary
- source
- test
- clang (not relevant on macOS)
- doc (not relevant on macOS)
- maintainer (not relevant on macOS)
