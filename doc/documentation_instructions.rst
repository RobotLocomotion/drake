.. _documentation-generation-instructions:

*************************************
Documentation Generation Instructions
*************************************

*Note: Before proceeding, please*
:ref:`build Drake from source <build_from_source>`. This is necessary because
otherwise the various build targets mentioned below will not exist.

This section contains instructions on how to generate Drake's documentation.
This includes both API documentation
(`C++ <http://drake.mit.edu/doxygen_cxx/index.html>`_),
which uses `Doxygen <http://www.stack.nl/~dimitri/doxygen/>`_, and
`Drake's website <http://drake.mit.edu>`_, which
uses `Sphinx <http://www.sphinx-doc.org/en/stable/index.html>`_.

.. _documentation-generation-instructions-bazel:

When using Bazel
================

To generate the website (Sphinx) documentation::

    $ bazel run //doc:serve_sphinx

This will rebuild the website content and serve it to your web browser for
preview using https://docs.python.org/2/library/webbrowser.html.

To merely compile the website into ``bazel-genfiles/doc/sphinx.zip``
without launching a preview::

    $ bazel build //doc:sphinx.zip

To generate the Doxygen documentation::

    $ cd drake-distro
    $ doc/doxygen.py [--quick]

To view the generated documentation, open using a web browser to
``drake-distro/build/drake/doc/doxygen_cxx/html/index.html``
