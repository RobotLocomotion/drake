---
title: Documentation Generation Instructions
---

Documentation generation and preview as described in this document are
supported on Ubuntu only.

*Note: Before proceeding, please*
[build Drake from source](/from_source.html). This is necessary because
otherwise the various build targets mentioned below will not exist.

This section contains instructions on how to generate Drake's documentation,
which uses a combination of
[Sphinx](http://www.sphinx-doc.org/en/stable/index.html) and
[Doxygen](https://www.stack.nl/~dimitri/doxygen/).
This includes API documentation
([C++](https://drake.mit.edu/doxygen_cxx/index.html) and
[Python](https://drake.mit.edu/pydrake/index.html)) and
[Drake’s website](https://drake.mit.edu/).

#### When using Bazel

First, install the appropriate prerequisites with the ``--with-doc-only`` command
line option, e.g., :

```
$ ./setup/ubuntu/install_prereqs.sh --with-doc-only
```

Then, to generate the website and serve it locally with
[webbrowser](https://docs.python.org/2/library/webbrowser.html):

```
$ bazel run //doc:serve_sphinx [-- --browser=false]
```

The contents of the website are also available via

```bazel build //doc:sphinx.zip```.

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
