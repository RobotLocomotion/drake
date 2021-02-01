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
available as a required external in the Bazel build.
