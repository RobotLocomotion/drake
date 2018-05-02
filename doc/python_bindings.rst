.. _python-bindings:

***********************
Using Drake from Python
***********************

A limited subset of the Drake C++ functionality is available from Python. The
Drake Python bindings are generated using `pybind11
<https://github.com/pybind/pybind11>`_, which means that every function or
class which is exposed to C++ has been explicitly enumerated in one of the
source files inside the ``bindings/pydrake`` folder. These bindings are
installed as a single package called ``pydrake``.

Python 2.7 is currently the only supported version for these bindings.

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

    cmake -DWITH_GUROBI=ON -DWITH_MOSEK=ON ../drake

Using the Python Bindings
-------------------------

To use the Drake Python bindings, follow the build steps above or ensure that
you have installed Drake appropriately. You will also need to have your
``PYTHONPATH`` configured correctly.

As an example, continuing from the code snippets from above:

.. code-block:: shell

    cd drake-build
    export PYTHONPATH=${PWD}/install/lib/python2.7/site-packages:${PYTHONPATH}

To check this:

.. code-block:: shell

    python -c 'import pydrake; print(pydrake.__file__)'

.. note::

    If you are using macOS, you must ensure that you are using the ``python2``
    executable to run these scripts.

    If you would like to use ``jupyter``, then be sure to install it via
    ``pip2 install jupyter`` (*not* ``brew install jupyter``) to ensure that it
    uses the correct ``PYTHONPATH``.

    ..
        Developers: Ensure this is synchronized with the steps in
        ``install_prereqs_user_environment.sh``.

.. note::

    If you are using Gurobi, you must either have it installed in the suggested
    location under ``/opt/...`` mentioned in :ref:`gurobi`, or you must ensure
    that you define the ``${GUROBI_PATH}`` environment variable, or specify
    ``${GUROBI_INCLUDE_DIR}`` via CMake.

What's Available from Python
----------------------------

The most up-to-date demonstrations of what can be done using ``pydrake`` are
the ``pydrake`` unit tests themselves. You can see all of them inside the
``drake/bindings/python/pydrake/**/test`` folders in the Drake source code.

Here's an example snippet of code from ``pydrake``:

..
    Developers: Ensure these snippets are synchronized with
    ``//bindings/pydrake:all_test``

.. code-block:: python

    from pydrake.common import FindResourceOrThrow
    from pydrake.multibody.rigid_body_plant import RigidBodyPlant
    from pydrake.multibody.rigid_body_tree import RigidBodyTree
    from pydrake.systems.analysis import Simulator

    tree = RigidBodyTree(
        FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf"))
    simulator = Simulator(RigidBodyPlant(tree))

If you are prototyping code in a REPL environment (such as IPython / Jupyter)
and to reduce the number of import statements, consider using ``pydrake.all`` to
import a subset of symbols from a flattened namespace or import all modules
automatically. If you are writing non-prototype code, avoid using
``pydrake.all``; for more details, see ``help(pydrake.all)``.

In all cases, try to avoid using ``from pydrake.all import *``, as it may
introduce symbol collisions that are difficiult to debug.

An example of importing symbols directly from ``pydrake.all``:

.. code-block:: python

    from pydrake.all import (
        FindResourceOrThrow, RigidBodyPlant, RigidBodyTree, Simulator)

    tree = RigidBodyTree(
        FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf"))
    simulator = Simulator(RigidBodyPlant(tree))

An alternative is to use ``pydrake.all`` to import all modules, but then
explicity refer to each symbol:

.. code-block:: python

    import pydrake.all

    tree = pydrake.multibody.rigid_body_tree.RigidBodyTree(
        pydrake.common.FindResourceOrThrow(
            "drake/examples/pendulum/Pendulum.urdf"))
    simulator = pydrake.systems.analysis.Simulator(
        pydrake.multibody.rigid_body_plant.RigidBodyPlant(tree))

Documentation
=============

There is not yet a comprehensive API documentation for the Python bindings
(tracked by `#7914 <https://github.com/RobotLocomotion/drake/issues/7914>`_).

In general, the Python API should be close to the
`C++ API <doxygen_cxx/index.html#://>`_. There are some exceptions:

C++ Template Instantiations in Python
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

When you define a general class template, e.g.
``template <typename T> class Value``, something like ``Value<std::string>`` is
called the instantiation.

For certain C++ templated types, they are exposed in Pythons also as templates;
the parameter types (in this case, ``T``) are the Python-equivalent types to the
C++ type. Some examples:

+---------------------------------+--------------------------------------+
| C++                             | Python                               |
+=================================+======================================+
| ``std::string``                 | ``str``                              |
+---------------------------------+--------------------------------------+
| ``double``                      | ``float``, ``np.double``,            |
|                                 | ``np.float64``, ``ctypes.c_double``  |
+---------------------------------+--------------------------------------+
| ``drake::AutoDiffXd``           | ``pydrake.autodiffutils.AutoDiffXd`` |
+---------------------------------+--------------------------------------+
| ``drake::symbolic::Expression`` | ``pydrake.symbolic.Expression``      |
+---------------------------------+--------------------------------------+

Thus, the instantiation ``Value<std::string>`` will be bound in Python as
``Value[str]``.

Scalar Types
^^^^^^^^^^^^

Most classes in the Systems framework and in the multibody dynamics
computational framework are templated on a scalar type, ``T``.
For convenience (and backwards compatibility) in Python, a slightly different
binding convention is used.

For example, ``Adder<T>`` is a Systems primitive which has a user-defined
number of inputs and outputs a single port which is the sum of all of the
inputs.

In C++, you would access the instantiations using ``Adder<double>``,
``Adder<AutoDiffXd>``, and ``Adder<Expression>`` for common scalar types.

In Python, ``Adder`` actually refers to the "default" instantiation, the
``Adder<double>`` C++ class. To access other instantiations, you should add an
``_`` to the end of the C++ class name to get the Python template and then
provide the parameters in square braces, ``[...]``. In this example, you should
use ``Adder_[T]``.

To illustrate, you can print out the string representations of ``Adder``,
``Adder_``, and some of its instantiations in Python:

.. code-block:: pycon

    >>> from pydrake.systems.primitives import Adder, Adder_
    >>> print(Adder)
    <class 'pydrake.systems.primitives.Adder_[float]'>
    >>> print(Adder_)
    <TemplateClass pydrake.systems.primitives.Adder_>
    >>> from pydrake.autodiffutils import AutoDiffXd
    >>> from pydrake.symbolic import Expression
    >>> print(Adder_[float])
    <class 'pydrake.systems.primitives.Adder_[float]'>
    >>> print(Adder_[AutoDiffXd])
    <class 'pydrake.systems.primitives.Adder_[AutoDiffXd]'>
    >>> print(Adder_[Expression])
    <class 'pydrake.systems.primitives.Adder_[Expression]'>

Additionally, you may convert an instance (if the conversion is available) using
``System_[T].ToAutoDiffXd`` and ``System_[T].ToSymbolic``:

.. code-block:: pycon

    >>> adder = Adder(num_inputs=1, size=1)
    >>> print(adder)
    <pydrake.systems.primitives.Adder_[float] object at 0x...>
    >>> print(adder.ToAutoDiffXd())
    <pydrake.systems.primitives.Adder_[AutoDiffXd] object at 0x...>
    >>> print(adder.ToSymbolic())
    <pydrake.systems.primitives.Adder_[Expression] object at 0x...>

For Developers
--------------

If you are developing Python bindings, please see the Doxygen page for
`Python Bindings <http://drake.mit.edu/doxygen_cxx/python_bindings.html>`_.
This provides information on programming conventions as well as tips for
debugging.
