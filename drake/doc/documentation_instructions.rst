.. _documentation-generation-instructions:

*************************************
Documentation Generation Instructions
*************************************

*Note: Before proceeding, please*
:ref:`build Drake from source <build_from_source>`.

This section contains instructions on how to generate Drake's documentation.
This includes both API documentation
(`C++ <http://drake.mit.edu/doxygen_cxx/index.html>`_ and
`MATLAB <http://drake.mit.edu/doxygen_matlab/index.html>`_),
which uses `Doxygen <http://www.stack.nl/~dimitri/doxygen/>`_, and
`Drake's website <http://drake.mit.edu>`_, which
uses `Sphinx <http://www.sphinx-doc.org/en/stable/index.html>`_.

Drake's documentation is built using the ``documentation`` build target::

    $ cd drake-distro/build/drake
    $ [make|ninja] documentation

To view the generated documentation, open the following files using a web
browser:

- Drake website: ``drake-distro/build/drake/doc/sphinx/index.html``
- Doxygen C++ website: ``drake-distro/build/drake/doc/doxygen_cxx/html/index.html``
- Doxygen Matlab website: ``drake-distro/build/drake/doc/doxygen_matlab/html/index.html``
