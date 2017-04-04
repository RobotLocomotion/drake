.. _python-bindings:

***********************
Using Drake from Python
***********************

A limited subset of the Drake C++ functionality is available from Python. The Drake Python bindings are generated using `pybind11 <https://github.com/pybind/pybind11>`_, which means that every function or class which is exposed to C++ has been explicitly enumerated in one of the source files inside the ``drake/bindings/pybind11`` folder. These bindings are installed as a single package called ``pydrake``.

Using the Python Bindings
=========================

To use the Drake Python bindings, you need to build Drake and then ensure that Python knows where to find the Drake bindings. When building with CMake, the ``pydrake`` package will be installed by default to ``build/install/lib/python2.7/dist-packages``. You will need to add this folder to your ``PYTHONPATH`` environment variable. For example, if you have Drake installed in ``/home/user/drake-distro/build/install``, then you should do::

    export PYTHONPATH="$PYTHONPATH:/home/user/drake-distro/build/install/lib/python2.7/dist-packages"

With your Python path configured appropriately, you should be able to import pydrake::


    $ python
    Python 2.7.6 (default, Oct 26 2016, 20:30:19)
    [GCC 4.8.4] on linux2
    Type "help", "copyright", "credits" or "license" for more information.
    >>> import pydrake
    >>>

What's Available from Python
============================

The most up-to-date demonstrations of what can be done using ``pydrake`` are the ``pydrake`` unit tests themselves. You can see all of them inside the ``drake/bindings/python/pydrake/test`` folder in the Drake source code.
