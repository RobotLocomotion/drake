.. _documentation-generation-instructions:

*************************************
Documentation Generation Instructions
*************************************

Documentation generation and preview as described in this document are
supported on Ubuntu only.

Before getting started, install the appropriate prerequisites with the
``--with-doc-only`` command line option, e.g., ::

    $ ./setup/ubuntu/install_prereqs.sh --with-doc-only

Sphinx and Doxygen
==================

*Note: Before proceeding, please*
:ref:`build Drake from source <build_from_source>`. This is necessary because
otherwise the various build targets mentioned below will not exist.

This section contains instructions on how to generate Drake's documentation,
which uses a combination of
`Sphinx <http://www.sphinx-doc.org/en/stable/index.html>`_ and
`Doxygen <https://www.stack.nl/~dimitri/doxygen/>`_.
This includes API documentation
(`C++ <https://drake.mit.edu/doxygen_cxx/index.html>`_ and
`Python <https://drake.mit.edu/pydrake/index.html>`_) and
`Drake's website <https://drake.mit.edu>`_.

To generate the website and serve it locally with
`webbrowser <https://docs.python.org/2/library/webbrowser.html>`_::

    $ bazel run //doc:serve_sphinx [-- --browser=false]

The contents of the website are also available via
``bazel build //doc:sphinx.zip``.

To generate the C++ API documentation::

    $ cd drake
    $ bazel build //doc:doxygen
    $ bazel-bin/doc/doxygen [options]

    $ bazel-bin/doc/doxygen --help  # To learn about the possible options.

To generate the Python API documentation::

    $ bazel run //bindings/pydrake/doc:serve_sphinx [-- --browser=false]

The contents of the Python API documentation are also available via
``bazel build //bindings/pydrake/doc:sphinx.zip``.

Jekyll
======

*Note: Jekyll documentation is a work in progress and is not published on
the drake.mit.edu website yet. You can still preview it locally.*

It is *not* necessary to build Drake prior to running either command below.

To serve page locally at ``http://127.0.0.1:<n>``::

    $ bazel run //doc:serve_jekyll [-- --default_port <n>]

If not specified, the default port is 8000.

To create output in the specified out_dir::

    $ bazel run //doc:gen_jekyll -- --out_dir

The output directory must not already exist.
