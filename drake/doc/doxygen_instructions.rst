.. _doxygen-instructions:

********************
Doxygen Instructions
********************

This section constains instructions on how to use Doxygen with Drake.

.. _doxygen-style-guide:

Doxygen Style Guide
===================

Comming soon. See issue
`#2051 <https://github.com/RobotLocomotion/drake/issues/2051>`_.

.. _doxygen-generation:

Doxygen Website Generation
==========================

To generate Drake's Doxygen website, execute the following::

    $ cd [build artifacts directory]
    $ cmake -DBUILD_DOCUMENTATION=ON ..
    $ make

The generated website will be in
``[build artifacts directory]/doc/doxygen_cxx/html/``.
To view the generated website, open ``index.html`` using your favorite web
browser.

Note that if you're building Drake in-source (i.e., within the ``pod-build``
directory), the ``[build artifacts directory]`` is typically
``[drake distro]/drake/pod-build/``.

To disable documentation generation, execute::

    $ cd [build artifacts directory]
    $ cmake -DBUILD_DOCUMENTATION=OFF ..
    $ make
