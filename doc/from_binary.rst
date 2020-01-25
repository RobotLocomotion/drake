.. _binary-installation:

***********************************
Binary installation (macOS, Ubuntu)
***********************************

.. _nightly-releases:

Nightly releases
================

There are `binary packages <https://github.com/RobotLocomotion/drake/issues/1766#issuecomment-318955338>`_ of Drake available at:

- https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-bionic.tar.gz
- https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-mac.tar.gz
- :samp:`https://drake-packages.csail.mit.edu/drake/nightly/drake-{yyyymmdd}-{bionic|mac}.tar.gz`.
    - Example: https://drake-packages.csail.mit.edu/drake/nightly/drake-20191026-bionic.tar.gz

Note that Drake no longer supports Ubuntu 16.04 (Xenial), but you can still
download this package from at or before October 26, 2019 as follows:

- https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-xenial.tar.gz
- https://drake-packages.csail.mit.edu/drake/nightly/drake-20191026-xenial.tar.gz

Individual packages are archived two years from their date of creation.

Example usages of these binaries are shown in this `example CMake project <https://github.com/RobotLocomotion/drake-external-examples/tree/master/drake_cmake_installed>`_.
For the compilers used to produce these releases, see :ref:`binary-packages`.
If you are unsure of which approach to use, we suggest you :ref:`build from source <build_from_source>`
instead.

For using Python bindings, see :ref:`Binary Installation for Python
<python-bindings-binary>`.

You may also use these binary releases in Docker images. See :ref:`docker_hub`
for more information.

Drake binary releases incorporate a pre-compiled version of
`SNOPT <https://ccom.ucsd.edu/~optimizers/solvers/snopt/>`_ as part of the
`Mathematical Program toolbox <https://drake.mit.edu/doxygen_cxx/group__solvers.html>`_.
Thanks to Philip E. Gill and Elizabeth Wong for their kind support.

Drake maintainers may build "experimental" packages on demand using Jenkins by
following :ref:`these instructions <building-binary-packages-on-demand>`.

Historical Note
===============

Older releases were built around substantial MATLAB support, and are
described on :ref:`this release notes page <older_releases>`.
