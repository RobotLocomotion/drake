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

At the moment, only the website (Sphinx) documentation is supported via the
Bazel build::

    $ bazel run //drake/doc:serve_sphinx

This will rebuild the website content and serve it to your web browser for
preview using https://docs.python.org/2/library/webbrowser.html.

To merely compile the website into ``bazel-genfiles/drake/doc/sphinx.zip``
without launching a preview::

    $ bazel build //drake/doc:sphinx.zip

.. _documentation-generation-instructions-cmake:

When using CMake
================

Drake's documentation is built using the ``documentation`` build target.
After building Drake like normal, execute::

    $ cd drake-distro/build/drake
    $ [make|ninja] documentation

To view the generated documentation, open the following files using a web
browser:

- Drake website: ``drake-distro/build/drake/doc/sphinx/index.html``
- Doxygen C++ website: ``drake-distro/build/drake/doc/doxygen_cxx/html/index.html``

If you're using ``ninja``, there are two additional build targets that allow you
to build just the Sphinx website or C++ Doxygen website::

    $ cd drake-distro/build/drake
    $ ninja doc/doxygen_cxx_output
    $ ninja doc/sphinx_output
