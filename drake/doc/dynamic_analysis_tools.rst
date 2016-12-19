.. _dynamic_analysis_tools:

**********************
Dynamic Analysis Tools
**********************

.. contents:: `Table of contents`
   :depth: 3
   :local:

.. _dynamic_analysis_tools-introduction:

Introduction
============

Several dynamic analysis tools may be configured within CMake and CTest to run
as part of testing. The dynamic analysis tools test only C and C++ code. MATLAB
and Python code is not tested, and external dependencies that require MATLAB or
Python are disabled.

.. _dynamic_analysis_tools-sanitizers:

Sanitizers
==========

.. _dynamic_analysis_tools-address_sanitizer:

AddressSanitizer
----------------

`AddressSanitizer <https://github.com/google/sanitizers/wiki/AddressSanitizer>`_
is a fast memory error detector that is part of Clang 3.1 and above and GCC 4.8
and above. ::

    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Debug -DUSE_SANITIZER=Address ..
    cmake --build .
    cd drake
    ctest --extra-verbose .

.. _dynamic_analysis_tools-leak_sanitizer:

LeakSanitizer
-------------

`LeakSanitizer <https://github.com/google/sanitizers/wiki/AddressSanitizerLeakSanitizer>`_
is a fast memory leak detector that is part of AddressSanitizer. ::

    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Debug -DUSE_SANITIZER=Leak ..
    cmake --build .
    cd drake
    ctest --extra-verbose .

.. _dynamic_analysis_tools-memory_sanitizer:

MemorySanitizer
---------------

`MemorySanitizer <https://github.com/google/sanitizers/wiki/MemorySanitizer>`_
is a fast uninitialized memory read detector that is part of Clang 3.3 and
above. ::

    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Debug -DUSE_SANITIZER=Memory ..
    cmake --build .
    cd drake
    ctest --extra-verbose .

.. _dynamic_analysis_tools-thread_sanitizer:

ThreadSanitizer
---------------

`ThreadSanitizer <https://github.com/google/sanitizers/wiki/ThreadSanitizerCppManual>`_
is a fast data race detector that is part of Clang 3.2 and above and GCC 4.8 and
above. ::

    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Debug -DUSE_SANITIZER=Thread ..
    cmake --build .
    cd drake
    ctest --extra-verbose .

.. _dynamic_analysis_tools-undefined_behavior_sanitizer:

UndefinedBehaviorSanitizer
--------------------------

`UndefinedBehaviorSanitizer <http://clang.llvm.org/docs/UndefinedBehaviorSanitizer.html>`_
is a fast undefined behavior detector that is part of Clang 3.3 and above and
GCC 4.9 and above. ::

    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Debug -DUSE_SANITIZER=Undefined ..
    cmake --build .
    cd drake
    ctest --extra-verbose .

.. _dynamic_analysis_tools-valgrind_tools:

Valgrind Tools
==============

.. _dynamic_analysis_tools-cachegrind:

Cachegrind
----------

`Cachegrind <http://valgrind.org/docs/manual/cg-manual.html>`_ is a cache
and branch-prediction profiler that is part of the Valgrind tool suite. ::

    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Debug -DUSE_VALGRIND=Cachegrind ..
    cmake --build .
    cd drake
    ctest --extra-verbose --test-action MemCheck .

.. _dynamic_analysis_tools-helgrind:

Helgrind
----------

`Helgrind <http://valgrind.org/docs/manual/hg-manual.html>`_ is a thread error
detector that is part of the Valgrind tool suite. ::

    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Debug -DUSE_VALGRIND=Helgrind ..
    cmake --build .
    cd drake
    ctest --extra-verbose --test-action MemCheck .

.. _dynamic_analysis_tools-memcheck:

Memcheck
--------

`Memcheck <http://valgrind.org/docs/manual/mc-manual.html>`_ is a memory error
detector that is part of the Valgrind tool suite. ::

    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Debug -DUSE_VALGRIND=Memcheck ..
    cmake --build .
    cd drake
    ctest --extra-verbose --test-action MemCheck .
