---
title: Documentation Generation Instructions
---

This page contains instructions on how to generate Drake's documentation,
which uses a combination of
[Jekyll](https://jekyllrb.com/),
[Sphinx](http://www.sphinx-doc.org/en/stable/index.html), and
[Doxygen](https://www.stack.nl/~dimitri/doxygen/).
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

To preview changes locally at ``http://127.0.0.1:<n>``:

```
$ bazel run //doc:serve_jekyll [-- --default_port <n>]
```

If not specified, the default port is 8000.

To create output in the specified out_dir:

```
$ bazel run //doc:gen_jekyll -- --out_dir
```

The output directory must not already exist.

# Drake's API documentation

To generate the C++ API documentation:

```
$ cd drake
$ bazel build //doc:doxygen
$ bazel-bin/doc/doxygen [options]
$ bazel-bin/doc/doxygen --help  # To learn about the possible options.
```

To generate the Python API documentation:

```
$ bazel run //bindings/pydrake/doc:serve_sphinx [-- --browser=false]
```

The contents of the Python API documentation are also available via
``bazel build //bindings/pydrake/doc:sphinx.zip``.

# Continuous Integration

To check that the documentation will pass Drake's Jenkins CI builds:

```
$ bazel test //doc:manual_tests //doc:manual_binaries
```
