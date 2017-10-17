.. _unit-test-instructions:

************************************
Detailed Notes on Drake's Unit Tests
************************************

.. _introduction:

Introduction
============

Unit tests are essential for software maintainability. They demonstrate the
correctness of existing code and prevent future changes from breaking the
past functionality (i.e., regressions). They are the only
way for developers to inform Drake's
:ref:`Continuous Integration (CI) <continuous_integration_notes>` service how
the software is *supposed* to behave.

.. _unit-testing-frameworks:

Unit Testing Frameworks
=======================

.. _unit-testing-framework-cpp:

C++ Code
--------

Please use the
`Google Test Framework <https://github.com/google/googletest>`_. It is already
available as a required external in Drake's super-build level.

.. _unit-testing-framework-matlab:

MATLAB Code
-----------

Please write an example and add it as a test in the directory's
``CMakeLists.txt`` using ``drake_add_matlab_test()``.  ``ctest`` will consider the
test failed if it times out or calls ``error``.

.. _enable-long-running-unit-test:

Enabling Long Running Unit Tests
================================

Drake disables a number of long-running unit tests by default. To reduce
continuous integration turnaround time, these tests run on the build servers
post-submit, but not pre-submit. To enable these tests locally, execute::

    cd drake-distro/build/drake
    cmake -DLONG_RUNNING_TESTS=ON .

The last command above saves your new build option in
``[drake build artifacts directory]/CMakeCache.txt``.

Before the long-running unit tests can be executed, you need to re-build Drake.
``make`` users should run::

    cd drake-distro/build/drake
    make clean
    make

``ninja`` users should run::

    cd drake-distro/build/drake
    ninja clean
    ninja

Note that even after enabling long-running unit tests, certain tests may still
be disabled due to required dependencies not being available. The following
Github issue is tracking the documentation of unit tests dependencies:
https://github.com/RobotLocomotion/drake/issues/2733.

.. _run-all-unit-tests:

Running Every Unit Test
=======================

To run every enabled unit test, execute::

    cd drake-distro/build/drake
    ctest -VV

.. _list-all-unit-tests:

Finding a Specific Unit Test
============================

To find a specific unit test you can print a list of all enabled unit tests by
executing the following commands::

  cd drake-distro/build/drake
  ctest -N

If you have a clue about a particular unit test's name, you can pipe the output
of the `ctest -N` command to `grep`. One way to learn the name of a unit test is
to look at the ``CMakeLists.txt`` that adds the unit test to the build system.

.. _running-a-specific-test:

Running a Specific Test
=======================

Once you know the unit tests' name, you can run it by executing::

  cd drake-distro/build/drake
  ctest -VV -C [build mode] -R [test name]

where: ``[build mode]`` is the build mode, e.g., ``Debug``, ``RelWithDebInfo``,
or ``Release``, and ``[test name]`` is the name of the test exactly as printed
by ``ctest -N`` including, if any, the entire path to the test as printed on the
screen.

.. _example-running-unit-test:

Example: Find And Run A Unit Test Named `cascade_system`
========================================================

Find test::

  cd drake-distro/build/drake
  ctest -N | grep -i cascade

Run the test::

  ctest -VV -C RelWithDebInfo -R cascade_system_test

.. _enabling-assertions:

Enabling Assertions
===================

To enable Drake's assertions without switching to ``Debug`` mode,
define ``DRAKE_ENABLE_ASSERTS`` for the C++ pre-processor when you
configure the build, for example::

    cd drake-distro/build
    cmake .. -G Ninja -DCMAKE_CXX_FLAGS:STRING=-DDRAKE_ENABLE_ASSERTS
    ninja

We recommend turning on assertions in all of your builds during development.
