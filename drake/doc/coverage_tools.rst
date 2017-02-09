.. _coverage_tools:

**************
Coverage Tools
**************

.. contents:: `Table of contents`
   :depth: 2
   :local:

.. _coverage_tools-introduction:

Introduction
============

Several coverage tools may be configured within CMake to run as
part of or after testing.

.. _coverage_tools-gcov:

Gcov
====

Code coverage using `Gcov <https://gcc.gnu.org/onlinedocs/gcc/Gcov.html>`_ ::

    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE:STRING=Debug -DUSE_COVERAGE:STRING=Gcov ..
    cmake --build .
    cd drake
    ctest --extra-verbose .
    cmake --build . --target lcov

.. _coverage_tools-sanitizer:

Sanitizer
=========

Code coverage using
`SanitizerCoverage <http://clang.llvm.org/docs/SanitizerCoverage.html>`_ ::

    export CC=clang-3.9
    export CXX=clang++-3.9
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE:STRING=Debug -DUSE_COVERAGE:STRING=Sanitizer -DSANITIZER_COVERAGE_LEVEL=Edge ..
    cmake --build .
    cd drake
    ctest --extra-verbose .

Other options for `SANITIZER_COVERAGE_LEVEL` are `BasicBlock` and `Function`.
