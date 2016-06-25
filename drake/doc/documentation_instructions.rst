.. _documentation-generation-instructions:

*************************************
Documentation Generation Instructions
*************************************

This section contains instructions on how to generate Drake's documentation.
This includes both API documentation, which uses
`Doxygen <http://www.stack.nl/~dimitri/doxygen/>`_, and Drake's website, which
uses `Sphinx <http://www.sphinx-doc.org/en/stable/index.html>`_.

Drake's documentation is built using the ``documentation`` build target. It must
first be enabled prior to documentation generation. This process depends
on whether Drake is being built in-source our out-of-source, and which build
system is being used, e.g., ``make``, ``ninja``, or ``MSVC``. The following
subsections describe the process for the officially-supported configurations.
Please jump to the subsection that matches the build style and tool employed.

.. _documentation-in-source-make:

In-source Builds Using ``make``
===============================

Enable the ``documentation`` build target::

    $ cd drake-distro/drake/pod-build
    $ cmake -DBUILD_DOCUMENTATION=ON ..

Note that the ``documentation`` build target can later be disabled by
executing::

    $ cd drake-distro/drake/pod-build
    $ cmake -DBUILD_DOCUMENTATION=OFF ..

With the ``documentation`` build target enabled, execute the following commands
to build the documentation::

    $ cd drake-distro/drake/pod-build
    $ make documentation

To view the generated documentation, see:
:ref:`Viewing Generated Documentation <viewing-generated-documentation>`

.. _documentation-out-of-source-ninja:

Out-of-source Builds Using ``ninja``
====================================

Enable the ``documentation`` build target::

    $ cd drake-build/drake
    $ cmake -DBUILD_DOCUMENTATION=ON .

Note that the ``documentation`` build target can later be disabled by
executing::

    $ cd drake-build/drake
    $ cmake -DBUILD_DOCUMENTATION=OFF .

With the ``documentation`` build target enabled, execute the following commands
to build the documentation::

    $ cd drake-build/drake
    $ ninja documentation

To view the generated documentation, see:
:ref:`Viewing Generated Documentation <viewing-generated-documentation>`

.. _documentation-out-of-source-msvc:

Out-of-source Builds Using ``MSVC``
===================================

Enable the ``documentation`` build target::

    $ cd drake-build/drake
    $ cmake -DBUILD_DOCUMENTATION=ON .

Note that the ``documentation`` build target can later be disabled by
executing::

    $ cd drake-build/drake
    $ cmake -DBUILD_DOCUMENTATION=OFF .

In Visual Studio, there is a ``documentation`` target that's visible through the
Solution Explorer, as shown below:

.. image:: images/doxygen_instructions/visual_studio_build_targets.png
   :width: 500 px
   :align: center

To build the documentation, simply select and build the ``documentation`` target
in the IDE. Note that in Microsoft Visual Studio, the ``documentation`` target
is not built when building the other targets, meaning there is one less reason
to disable the ``documentation`` target.

To view the generated documentation, see:
:ref:`Viewing Generated Documentation <viewing-generated-documentation>`

.. _viewing-generated-documentation:

Viewing the Generated Documentation
===================================

To view the generated documentation, open the following files using your
favorite web browser::

- Drake website: ``drake-distro/drake/pod-build/doc/sphinx/index.html``
- Doxygen C++ website: ``drake-distro/drake/pod-build/doc/doxygen_cxx/html/index.html``
- Doxygen Matlab website: ``drake-distro/drake/pod-build/doc/doxygen_matlab/html/index.html``