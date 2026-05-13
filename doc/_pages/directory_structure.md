---
title: Directory Structure
---

The current directory structure in Drake is organized to (1) group functional
components, (2) limit dependencies, and (3) prevent circular dependencies.

# Experimental and Dev Directories

All code in Drake must adhere to the code standards described in the notes
[For Developers](/developers.html#developer-notes) and must be covered by
tests, unless the code lives in a subdirectory named ``experimental`` or
``dev``.

To promote rapid development of research ideas, experimental code may be placed
in a subdirectory named ``experimental`` or ``dev``. We do not enforce code
standards nor test coverage within those directories. For pull requests that
affect only ``experimental`` or ``dev`` directories, one feature review is
sufficient; platform review is not required.

If build or test targets in ``experimental`` or ``dev`` directories break, the
response from the on-call build cop will be to disable the offending target(s),
up to and including removing them from the build (and install) entirely. See
[Build Cop](/buildcop.html) for details.

The ``BUILD.bazel`` file for ``experimental`` and ``dev`` directories must live
within the directory itself, not a parent directory. (For example,
``foo/BUILD.bazel`` must **not** say ``srcs = ["experimental/bar.cc"],``;
instead, ``foo/experimental/BUILD.bazel`` must exist and must say
``srcs = ["bar.cc"],``.) This ensures that all ``experimental`` code has a
package name (``//foo/experimental``) that clearly denotes it as such.

Code in ``experimental`` or ``dev`` may only be depended-on by other code in
such directories. Stable code (outside of ``experimental`` and ``dev``) must not
``#include`` nor ``import`` code from ``experimental`` or ``dev``.

The distinction between ``experimental`` and ``dev`` is that code in
``experimental`` is installed by CMake, packaged as part of our nightly and
stable release binaries, and has API documentation on our website, but code in
``dev`` is neither installed, packaged, nor website-documented. (A corollary is
that code in ``dev`` cannot have Python bindings. If you need bindings, place
your code in ``experimental``, not ``dev``.)

Code in ``experimental`` must have ``experimental`` as part of its C++ namespace
and Python module name. It also must not be part of ``import pydrake.all``.

# Controlling Dependencies

We would like to avoid circular dependencies in the source code tree. For
example, some code in ``drake/systems`` depends on code in ``drake/solvers``, so
code in ``drake/solvers`` should *not* depend on ``drake/systems``.

The ``drake/examples`` directories are logically after all of the core libraries
in ``drake``. ``test`` directories located throughout the code are logically
last in the dependency tree -- they are allowed to depend on any of the other
directories. For example, ``test`` code **is** allowed to use models/code from
the ``examples`` directories to test core functionality, as long as that
dependency is explicitly declared in the build system rules for that test.
