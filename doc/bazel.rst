.. _bazel:

******************
Bazel build system
******************

The Bazel build system is officially supported for a subset of Drake on
Ubuntu Xenial and macOS.
For more information, see:

 * https://bazel.build/
 * https://github.com/RobotLocomotion/drake/issues/3129

Bazel Installation
==================

Follow Drake's
:ref:`platform-specific setup instructions <platform_specific_setup>`
to install Bazel.

Drake clone and platform setup
==============================

 - Start with a **git clone** of drake, per the :ref:`Getting Drake
   <getting_drake>` instructions.

 - Continue with the *"Mandatory platform specific instructions"* on the same
   page.

.. _using_bazel:

Developing Drake using Bazel
============================

To build or test Drake, run **bazel build** or **bazel test** with the desired
target label (and optional configuration options if desired).  We give some
typical examples below; for more reading about target patterns, see:
https://docs.bazel.build/versions/master/user-manual.html#target-patterns.

Under Bazel, Clang is the default compiler on all platforms, but command-line
options are available to use GCC on Ubuntu.

Cheat sheet for operating on the entire project::

  cd /path/to/drake
  bazel build //...                     # Build the entire project.
  bazel test //...                      # Build and test the entire project.
  bazel build --compiler=gcc-5 //...    # Build using gcc 5.x on Xenial.

- The "``//``" means "starting from the root of the project".
- The "``...``" means "everything including the subdirectories' ``BUILD`` files".

  - Contrast with, e.g., the "``bazel build common:*``" explained below, where
    only targets declared *directly* in ``drake/common/BUILD`` are compiled,
    and not the targets in ``drake/common/trajectories/BUILD``.  The "``*``"
    matches targets in that directory; the "``...``" also matches down into
    subdirectories.

You may use relative pathnames if your shell's working directory is not at the
project root::

  cd /path/to/drake/common
  bazel build ...                   # Build everything in common and its child subdirectories.
  bazel test ...                    # Test everything in common and its child subdirectories.
  bazel build //...                 # Build the entire project.
  bazel test //...                  # Build and test the entire project.

- As before, the "``...``" above means "everything including subdirectories".

  - In the first two lines we did not precede "``...``" with "``//``", so the
    search begins in the current directory (``common``) and not from the
    ``drake`` root.
  - In the second two lines we used the "``//``" prefix to specify the project
    root, so we're back to operating on the entire project even though
    ``common`` is still our shell's current working directory.

Cheat sheet for operating on specific portions of the project::

  cd /path/to/drake
  bazel build common/...                               # Build everything in common and its child subdirectories.
  bazel build common                                   # Build libcommon.
  bazel build common:polynomial                        # Build libpolynomial.
  bazel build common:*                                 # Build everything in common but NOT its children.

  bazel test common:polynomial_test                    # Run one test.
  bazel test -c dbg common:polynomial_test             # Run one test in debug mode.
  bazel test --config=memcheck common:polynomial_test  # Run one test under memcheck (valgrind).
  bazel test --config=fastmemcheck common:*            # Run common's tests under memcheck, with minimal recompiling.
  bazel test --config=asan common:polynomial_test      # Run one test under AddressSanitizer.
  bazel test --config=kcov common:polynomial_test      # Run one test under kcov (see instructions below).
  bazel build -c dbg common:polynomial_test && \
    gdb bazel-bin/common/polynomial_test               # Run one test under gdb.

  bazel test --config lint //...                       # Only run style checks; don't build or test anything else.

- The "``:``" syntax separates target names from the directory path of the
  ``BUILD`` file they appear in.  In this case, for example,
  ``drake/commmon/BUILD`` specifies ``cc_test(name = "polynomial_test")``.
- Note that the configuration switches (``-c`` and ``--config``) influence the
  entire command.  For example, running a test in ``dbg`` mode means that its
  prerequisite libraries are also compiled and linked in ``dbg`` mode.
- For the definitions of the "``--config``" options see ``drake/tools/bazel.rc``.

Debugging on macOS
------------------

On macOS, DWARF debug symbols are emitted to a ``.dSYM`` file.  The Bazel
``cc_binary`` and ``cc_test`` rules do not natively generate or expose this
file, so we have implemented a workaround in Drake, ``--config=apple_debug``.
This config turns off sandboxing, which allows a ``genrule`` to access the
``.o`` files and process them into a ``.dSYM``.  Use as follows::

  bazel build --config=apple_debug path/to/my:binary_or_test_dsym
  lldb ./bazel-bin/path/to/my/binary_or_test

For more information, see https://github.com/bazelbuild/bazel/issues/2537.

.. _buildifier:

Updating BUILD files
====================

Please use the "``buildifier``" tool to format edits to ``BUILD`` files (in the
same spirit as ``clang-format`` formatting C++ code)::

  cd /path/to/drake
  bazel-bin/tools/lint/buildifier --all         # Reformat all Bazel files.
  bazel-bin/tools/lint/buildifier common/BUILD  # Only reformat one file.

In most cases the ``bazel-bin/tools/lint/buildifier`` will already be compiled
by the time you need it.  In case it's absent, you can compile it via::

  cd /path/to/drake
  bazel build //tools/lint:buildifier

Proprietary Solvers
===================

The Drake Bazel build currently supports the following proprietary solvers:

 * Gurobi 8.0.0
 * MOSEK 8.1
 * SNOPT 7.2

.. _gurobi:

Gurobi 8.0.0
------------

Install on Ubuntu
~~~~~~~~~~~~~~~~~
1. Register for an account on https://www.gurobi.com.
2. Set up your Gurobi license file in accordance with Gurobi documentation.
3. ``export GRB_LICENSE_FILE=/path/to/gurobi.lic``.
4. Download ``gurobi8.0.0_linux64.tar.gz``
5. Unzip it.  We suggest that you use ``/opt/gurobi800`` to simplify working with Drake installations.
6. ``export GUROBI_PATH=/opt/gurobi800/linux64``

Install on macOS
~~~~~~~~~~~~~~~~
1. Register for an account on http://www.gurobi.com.
2. Set up your Gurobi license file in accordance with Gurobi documentation.
3. ``export GRB_LICENSE_FILE=/path/to/gurobi.lic``
4. Download and install ``gurobi8.0.0_mac64.pkg``.


To confirm that your setup was successful, run the tests that require Gurobi:

  ``bazel test --config gurobi --test_tag_filters=gurobi //...``

The default value of ``--test_tag_filters`` in Drake's ``bazel.rc`` excludes
these tests.  If you will be developing with Gurobi regularly, you may wish
to specify a more convenient ``--test_tag_filters`` in a local ``.bazelrc``.
See https://docs.bazel.build/versions/master/user-manual.html#bazelrc.

MOSEK 8.1
---------

The Drake Bazel build system downloads MOSEK 8.1.0.51 automatically.  No manual
installation is required.  Set the location of your license file as follows:

``export MOSEKLM_LICENSE_FILE=/path/to/mosek.lic``

To confirm that your setup was successful, run the tests that require MOSEK:

  ``bazel test --config mosek --test_tag_filters=mosek //...``

The default value of ``--test_tag_filters`` in Drake's ``bazel.rc`` excludes
these tests.  If you will be developing with MOSEK regularly, you may wish
to specify a more convenient ``--test_tag_filters`` in a local ``.bazelrc``.
See https://docs.bazel.build/versions/master/user-manual.html#bazelrc.

SNOPT 7.2
---------

Drake provides two mechanisms to include the SNOPT sources.  One mechanism is
to provide your own SNOPT source archive.  The other mechanism is via access to
a private RobotLocomotion git repository.

Using your own source archive
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Download the SNOPT sources from the distributor in ``.tar.gz`` format (e.g.,
   named ``snopt7.5-1.4.tar.gz``).
2. ``export SNOPT_PATH=/home/username/Downloads/snopt7.5-1.4.tar.gz``

Using the RobotLocomotion git repository
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Obtain access to the private RobotLocomotion/snopt GitHub repository.
2. `Set up SSH access to github.com <https://help.github.com/articles/adding-a-new-ssh-key-to-your-github-account/>`_.

The build will attempt to use this mechanism anytime SNOPT is enabled and a
source archive has not been specified.

Test the build (for either mechanism)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To confirm that your setup was successful, run the tests that require SNOPT:

  ``bazel test --config snopt --test_tag_filters=snopt //...``

The default value of ``--test_tag_filters`` in Drake's ``bazel.rc`` excludes
these tests.  If you will be developing with SNOPT regularly, you may wish
to specify a more convenient ``--test_tag_filters`` in a local ``.bazelrc``.
See https://docs.bazel.build/versions/master/user-manual.html#bazelrc.

Optional Tools
==============

The Drake Bazel build system has integration support for some optional
development tools:

 * kcov -- test coverage analysis

kcov
----

``kcov`` can analyze coverage for any binary that contains DWARF format
debuggging symbols, and produce nicely formatted browse-able coverage reports.
It is supported on Ubuntu and macOS only.  Install ``kcov`` from source
following the instructions here: :ref:`Building kcov <building-kcov>`.

To analyze test coverage, run the tests under ``kcov``::

  bazel test --config kcov //...

Note that it disables compiler-optimization (``-O0``) to have a better and more
precise coverage report.  If you have trouble with kcov and unoptimized programs,
you can turn it back on by also supplying ``--copt -O2``.

The coverage report is written to the ``drake/bazel-kcov`` directory.  To
view it, browse to ``drake/bazel-kcov/index.html``.

.. toctree::
   :hidden:

   building_kcov
