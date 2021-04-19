---
title: Documentation Generation Instructions
---

This page contains instructions on how to generate Drake's documentation,
which uses a combination of
[Jekyll](https://jekyllrb.com/),
[Sphinx](http://www.sphinx-doc.org/en/stable/index.html), and
[Doxygen](https://www.doxygen.nl/index.html).
This includes [Drake's main website](https://drake.mit.edu/) and
API documentation
([C++](https://drake.mit.edu/doxygen_cxx/index.html) and
[Python](https://drake.mit.edu/pydrake/index.html)).


Documentation generation and preview as described in this document are
supported on Ubuntu only.

Before getting started, install Drake's prerequisites with the
``--with-doc-only`` command line option, i.e.:

```
$ ./setup/ubuntu/install_prereqs.sh --with-doc-only
```

# Drake's main website

To preview changes locally:

```
$ bazel run //doc:pages -- --serve
```

To create output in the specified out_dir:

```
$ bazel run //doc:pages -- --out_dir
```

The output directory must not already exist.

# Drake's API documentation

To generate the C++ API documentation:

```
$ bazel run //doc/doxygen_cxx:build -- --serve
```

If you only need to preview a subset of the API, you can speed up the
rebuild by listing a subset of module names on the command line:

```
$ bazel run //doc/doxygen_cxx:build -- --serve drake/math
```

You may also specify ``--quick`` to skip the image generation.

To generate the Python API documentation:

```
$ bazel run //doc/pydrake:build -- --serve
```

If you only need to preview a subset of the API, you can speed up the
rebuild by listing a subset of module names on the command line:

```
$ bazel run //doc/pydrake:build -- --serve pydrake.common pydrake.math
````

# Style Guide

To locally preview the Drake Style Guide:

```
$ bazel run //doc/styleguide:build -- --serve
```

To preview a local branch of the styleguide, set the
[local_repository_override](https://github.com/RobotLocomotion/drake/blob/master/tools/workspace/README.md#exploring-github_archive-changes-from-a-local-clone)
option in ``drake/tools/workspace/styleguide/`` before running the preview.

# Continuous Integration

To check that the documentation will pass Drake's Jenkins CI builds:

```
$ bazel test //doc:manual_tests //doc:manual_binaries
```
