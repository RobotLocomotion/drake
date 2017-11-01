.. _python-bindings:

***********************
Using Drake from Python
***********************

A limited subset of the Drake C++ functionality is available from Python. The Drake Python bindings are generated using `pybind11 <https://github.com/pybind/pybind11>`_, which means that every function or class which is exposed to C++ has been explicitly enumerated in one of the source files inside the ``drake/bindings/pybind11`` folder. These bindings are installed as a single package called ``pydrake``.

Using the Python Bindings
=========================

To use the Drake Python bindings, you need to build Drake and then ensure that Python knows where to find the Drake bindings.

With your ``PYTHONPATH`` configured appropriately, you should be able to import pydrake::

    $ python
    Python 2.7.12 (default, Nov 19 2016, 06:48:10)
    [GCC 5.4.0 20160609] on linux2
    Type "help", "copyright", "credits" or "license" for more information.
    >>> import pydrake
    >>>

What's Available from Python
============================

The most up-to-date demonstrations of what can be done using ``pydrake`` are the ``pydrake`` unit tests themselves. You can see all of them inside the ``drake/bindings/python/pydrake/test`` folder in the Drake source code.
