.. _python-bindings:

***********************
Using Drake from Python
***********************

A limited subset of the Drake C++ functionality is available from Python. The
Drake Python bindings are generated using `pybind11
<https://github.com/pybind/pybind11>`_, which means that every function or
class which is exposed to C++ has been explicitly enumerated in one of the
source files inside the ``drake/bindings/pybind11`` folder. These bindings are
installed as a single package called ``pydrake``.

Building the Python Bindings
----------------------------

To use the Python bindings from Drake externally, we recommend using CMake.
As an example:

.. code-block:: shell

    git clone https://github.com/RobotLocomotion/drake.git
    mkdir drake-build
    cd drake-build
    cmake ../drake
    make -j

Please note the additional CMake options which affect the Python bindings:

*   ``-DWITH_GUROBI={ON, [OFF]}`` - Build with Gurobi enabled.
*   ``-DWITH_MOSEK={ON, [OFF]}`` - Build with MOSEK enabled.
*   ``-DWITH_SNOPT={ON, [OFF]}`` - Build with SNOPT enabled.

``{...}`` means a list of options, and the option surrounded by ``[...]`` is
the default option. An example of building ``pydrake`` with both Gurobi and
MOSEK, without building tests:

.. code-block:: shell

    cmake -DWITH_GUROBI=ON -DWITH_MOSEK=ON -DBUILD_TESTING=OFF ../drake

Using the Python Bindings
=========================

To use the Drake Python bindings, you need to build Drake and then ensure that
Python knows where to find the Drake bindings.

With your ``PYTHONPATH`` configured appropriately, you should be able to import
pydrake. From the example above:

.. code-block:: shell

    cd drake-build
    export PYTHONPATH=${PWD}/install/lib/python2.7/site-packages:${PYTHONPATH}

To check this:

.. code-block:: shell

    python -c 'import pydrake; print(pydrake.__file__)'

What's Available from Python
============================

The most up-to-date demonstrations of what can be done using ``pydrake`` are
the ``pydrake`` unit tests themselves. You can see all of them inside the
``drake/bindings/python/pydrake/test`` folder in the Drake source code.
