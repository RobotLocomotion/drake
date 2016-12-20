.. _lint_static_analysis_tools:

******************************
Lint and Static Analysis Tools
******************************

.. contents:: `Table of contents`
   :depth: 2
   :local:

.. _lint_static_analysis_tools-introduction:

Introduction
============

Several lint and static analysis tools may be configured within CMake to run as
part of a build.

.. _lint_static_analysis_tools-clang_tidy:

clang-tidy
==========

`clang-tidy <http://clang.llvm.org/extra/clang-tidy/>`_ is a linter and static
analyzer based on Clang. It is supported by CMake 3.6 and above. ::

    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE:STRING=Debug -DUSE_CLANG_TIDY:BOOL=ON ..
    cmake --build .

.. _lint_static_analysis_tools-include_what_you_use:

include-what-you-use
====================

`include-what-you-use <http://include-what-you-use.org>`_ is a tool based on
Clang to identify unnecessary and/or missing includes. ::

    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE:STRING=Debug -DUSE_INCLUDE_WHAT_YOU_USE:BOOL=ON ..
    cmake --build .

.. _lint_static_analysis_tools-link_what_you_use:

link-what-you-use
=================

`link-what-you-use <https://cmake.org/cmake/help/latest/prop_tgt/LINK_WHAT_YOU_USE.html>`_
uses ldd to identify libraries that provide no symbols to a executable or
shared library to which they are being linked. It is supported by CMake 3.7 and
above. ::

    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE:STRING=Debug -DUSE_LINK_WHAT_YOU_USE:BOOL=ON ..
    cmake --build .
