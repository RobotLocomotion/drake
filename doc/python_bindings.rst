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
=========================

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

    If you are on Mac, you must ensure that you have Homebrew Python installed,
    and are using ``python2`` from Homebrew Python to execute these scripts.
    You may do this by either explicitly using ``python2`` on the command line,
    or follow the instructions from ``brew info python``::

        This formula installs a python2 executable to /usr/local/bin.
        If you wish to have this formula's python executable in your PATH then
        add the following to ~/.bash_profile:
          export PATH="/usr/local/opt/python/libexec/bin:$PATH"

    ..
        Developers: Ensure this is synchronized with the steps in
        ``install_prereqs_user_environment.sh``.

.. note::

    If you are using Gurobi, you must either have it installed in the suggested
    location under ``/opt/...`` mentioned in :ref:`gurobi`, or you must ensure
    that you define the ``${GUROBI_PATH}`` environment variable, or specify
    ``${GUROBI_INCLUDE_DIR}`` via CMake.

What's Available from Python
============================

The most up-to-date demonstrations of what can be done using ``pydrake`` are
the ``pydrake`` unit tests themselves. You can see all of them inside the
``drake/bindings/python/pydrake/test`` folder in the Drake source code.

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
