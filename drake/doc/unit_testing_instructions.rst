.. _unit-test-instructions:

***************************************
Detailed Notes on How to Run Unit Tests
***************************************

When you create a pull request, a number of unit and regression tests will
automatically run on Drake's build servers.  You can run those tests locally by
executing the commands shown below. Note, however, that there are many
computationally-demanding tests and running the entire test suite can take
several hours depending on your machine.

Your change must include unit tests that demonstrate the code's correctness and
protect it against regressions. These tests must pass on all
:ref:`supported platform configurations <supported-configurations>`.

**For C++ changes,** please use the
`Google Test Framework <https://github.com/google/googletest>`_. It is already
available as an external in Drake's super-build level.

**For MATLAB changes,** please write an example and add it as a test in the
directory's ``CMakeLists.txt`` using ``add_matlab_test()``.  ``ctest`` will
consider the test failed if it times out or calls ``error``.

.. _define-build-artifacts-directory:

Defining the Build Artifacts Directory
======================================

The instructions below frequently cite ``[drake build artifacts directory]``.
This is the directory where build artifacts are stored while compiling Drake.
The location of this directory depends on whether you're building Drake
in-source (i.e., using the ``pod-build`` directory within your clone of Drake's
repository) or out-of-source (i.e., within a directory that is outside of your
clone of Drake's repository).

For in-source builds, the build artifacts directory is typically
``[drake distro]/drake/pod-build``.

For out-of-source builds, the build artifacts directory is typically
``[out of source directory]/drake``.

.. _enable-long-running-unit-test:

Enabling Long Running Unit Tests
================================

Drake disables a number of long-running unit tests by default. To reduce
continuous integration turnaround time, these tests run on the build servers
post-submit, but not pre-submit. To enable these tests locally, execute::

    $ cd [drake build artifacts directory]
    $ cmake -DLONG_RUNNING_TESTS=ON .

The last command above saves your new build option in
``[drake build artifacts directory]/CMakeCache.txt``.

See :ref:`Defining the Build Artifacts Directory
<define-build-artifacts-directory>` above for the definition of
``[drake build artifacts directory]``.

.. _run-all-unit-tests:

Running Every Unit Test
=======================

To run every enabled unit test, execute::

    $ cd [drake build artifacts directory]
    $ ctest -VV

.. _list-all-unit-tests:

Finding a Specific Unit Test
============================

To find a specific unit test you can print a list of all enabled unit tests by
executing the following commands::

  $ cd [drake build artifacts directory]
  $ ctest -N

See :ref:`Defining the Build Artifacts Directory
<define-build-artifacts-directory>` above for the definition of
``[drake build artifacts directory]``.

If you have a clue about a particular unit test's name, you can pipe the output
of the `ctest -N` command to `grep`. One way to learn the name of a unit test is
to look at the ``CMakeLists.txt`` that adds the unit test to the build system.

.. _running-a-specific-test:

Running a Specific Test
=======================

Once you know the unit tests' name, you can run it by executing::

  $ ctest -VV -C [build mode] -R [test name]

where: ``[build mode]`` is the build mode, e.g., ``Debug``, ``RelWithDebInfo``,
or ``Release``, and ``[test name]`` is the name of the test exactly as printed
by ``ctest -N`` including, if any, the entire path to the test as printed on the
screen.

.. _example-running-unit-test:

Example: Find and run unit test named `cascade_system`
======================================================

Find test::

  $ cd [drake build artifacts directory]
  $ ctest -N | grep -i cascade

Run the test::

  $ ctest -VV -C RelWithDebInfo -R cascade_system_test

