.. _python-bindings:

***********************
Using Drake from Python
***********************

A limited subset of the Drake C++ functionality is available from Python. The Drake Python bindings are generated using `pybind11 <https://github.com/pybind/pybind11>`_, which means that every function or class which is exposed to C++ has been explicitly enumerated in one of the source files inside the ``drake/bindings/pybind11`` folder. These bindings are installed as a single package called ``pydrake``. 

Using the Python Bindings
=========================

To use the Drake Python bindings, you need to build Drake and then ensure that Python knows where to find the Drake bindings. When building with CMake, the ``pydrake`` package will be installed by default to ``build/install/lib/python2.7/dist-packages``. 

