.. _python-bindings:

***********************
Using Drake from Python
***********************

A substantial subset of the Drake C++ functionality is available from Python.
The Drake Python bindings are generated using `pybind11
<https://github.com/pybind/pybind11>`_, which means that every function or
class which is exposed to C++ has been explicitly enumerated in one of the
source files inside the ``bindings/pydrake`` folder. These bindings are
installed as a single package called ``pydrake``.

.. warning::
   The Python environment supplied by Anaconda is neither tested nor supported
   for building and using the Drake Python bindings.

.. _python-bindings-binary:

Installation
============

Before attempting installation, please review the
:ref:`supported configurations <supported-configurations>` to know what
versions of Python are supported for your platform.

Binary Installation for Python
------------------------------

First, download and extract an :ref:`available binary package
<binary-installation>`.

As an example, here is how to download and extract one of the latest releases
to ``/opt`` (where ``<platform>`` could be ``bionic``, ``focal``, or ``mac``):

.. code-block:: shell

    curl -o drake.tar.gz https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-<platform>.tar.gz
    rm -rf /opt/drake
    tar -xvzf drake.tar.gz -C /opt

Ensure that you have the system dependencies:

.. code-block:: shell

    /opt/drake/share/drake/setup/install_prereqs

Next, ensure that your ``PYTHONPATH`` is properly configured.

*Ubuntu 18.04 (Bionic):*

.. code-block:: shell

    export PYTHONPATH=/opt/drake/lib/python3.6/site-packages:${PYTHONPATH}

*Ubuntu 20.04 (Focal):*

.. code-block:: shell

    export PYTHONPATH=/opt/drake/lib/python3.8/site-packages:${PYTHONPATH}

*macOS:*

.. code-block:: shell

    export PYTHONPATH=/opt/drake/lib/python3.8/site-packages:${PYTHONPATH}

See :ref:`below <using-python-bindings>` for usage instructions.

Inside ``virtualenv``
^^^^^^^^^^^^^^^^^^^^^

At present, Drake is not installable via ``pip``. However, you can still
incorporate its install tree into a ``virtualenv``
`FHS <https://en.wikipedia.org/wiki/Filesystem_Hierarchy_Standard>`_-like
environment.

An example, where you should replace ``<venv_path>`` and ``<platform>``:

.. code-block:: shell

    # Setup drake, and run prerequisites.
    curl -o drake.tar.gz https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-<platform>.tar.gz
    mkdir -p <venv_path>
    tar -xvzf drake.tar.gz -C <venv_path> --strip-components=1
    # - You may need `sudo` here.
    <venv_path>/share/drake/setup/install_prereqs

    # Setup a virtualenv over the drake install.
    python3 -m virtualenv -p python3 <venv_path> --system-site-packages

.. note::

    You can extract Drake into an existing ``virtualenv`` tree if you have
    already run ``install_prereqs``; however, you should ensure that you have
    run ``install_prereqs``. Before you do this, you should capture / freeze
    your current requirements to reproduce your environment if there are
    conflicts.

To check if this worked, follow the instructions as
:ref:`shown below <using-python-bindings>`, but either:

*   Use ``<venv_path>/bin/python`` instead of ``python3``, or
*   Source ``<venv_path>/bin/activate`` in your current shell session.

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

You will also need to have your ``PYTHONPATH`` configured correctly.

*Ubuntu 18.04 (Bionic):*

.. code-block:: shell

    cd drake-build
    export PYTHONPATH=${PWD}/install/lib/python3.6/site-packages:${PYTHONPATH}

*Ubuntu 20.04 (Focal):*

.. code-block:: shell

    cd drake-build
    export PYTHONPATH=${PWD}/install/lib/python3.8/site-packages:${PYTHONPATH}

*macOS:*

.. code-block:: shell

    cd drake-build
    export PYTHONPATH=${PWD}/install/lib/python3.8/site-packages:${PYTHONPATH}

.. _using-python-bindings:

Using the Python Bindings
=========================

Check Installation
------------------

After following the above install steps, check to ensure you can import
``pydrake``.

.. code-block:: shell

    python3 -c 'import pydrake; print(pydrake.__file__)'

.. _using-python-mac-os-path:

.. note::

    If you are using Gurobi, you must either have it installed in the suggested
    location under ``/opt/...`` mentioned in :ref:`gurobi`, or you must ensure
    that you define the ``${GUROBI_HOME}`` environment variable, or specify
    ``${GUROBI_INCLUDE_DIR}`` via CMake.

.. _whats-available-from-python:

What's Available from Python
----------------------------

You should first browse the `Python API <pydrake/index.html#://>`_ to see what
modules are available. The most up-to-date high-level demonstrations of what
can be done using ``pydrake`` are in Drake's :ref:`Tutorials <tutorials>` and
the `Underactuated Robotics Textbook <http://underactuated.mit.edu/>`_.

You can also see lower-level usages of the API in the ``pydrake`` unit tests
themselves, which you can find inside of the
``drake/bindings/python/pydrake/**/test`` folders in the Drake source code.

Here's an example snippet of code from ``pydrake``:

..
    Developers: Ensure these snippets are synchronized with
    ``//bindings/pydrake:all_test``

.. code-block:: python

    from pydrake.common import FindResourceOrThrow
    from pydrake.multibody.parsing import Parser
    from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
    from pydrake.systems.analysis import Simulator
    from pydrake.systems.framework import DiagramBuilder

    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
    Parser(plant).AddModelFromFile(
        FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf"))
    plant.Finalize()
    diagram = builder.Build()
    simulator = Simulator(diagram)

If you are prototyping code in a REPL environment (such as IPython / Jupyter)
and to reduce the number of import statements, consider using ``pydrake.all`` to
import a subset of symbols from a flattened namespace or import all modules
automatically. If you are writing non-prototype code, avoid using
``pydrake.all``; for more details, see ``help(pydrake.all)``.

In all cases, try to avoid using ``from pydrake.all import *``, as it may
introduce symbol collisions that are difficult to debug.

The above example, but using ``pydrake.all``:

.. code-block:: python

    from pydrake.all import (
        AddMultibodyPlantSceneGraph, DiagramBuilder, FindResourceOrThrow,
        Parser, Simulator)

    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
    Parser(plant).AddModelFromFile(
        FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf"))
    plant.Finalize()
    diagram = builder.Build()
    simulator = Simulator(diagram)

An alternative is to use ``pydrake.all`` to import all modules, but then
explicitly refer to each symbol:

.. code-block:: python

    import pydrake.all

    builder = pydrake.systems.framework.DiagramBuilder()
    plant, _ = pydrake.multibody.plant.AddMultibodyPlantSceneGraph(builder, 0.0)
    pydrake.multibody.parsing.Parser(plant).AddModelFromFile(
        pydrake.common.FindResourceOrThrow(
            "drake/examples/pendulum/Pendulum.urdf"))
    plant.Finalize()
    diagram = builder.Build()
    simulator = pydrake.systems.analysis.Simulator(diagram)

Differences with C++ API
------------------------

In general, the `Python API <pydrake/index.html#://>`_ should be close to the
`C++ API <doxygen_cxx/index.html#://>`_. There are some exceptions:

C++ Class Template Instantiations in Python
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

C++ Function and Method Template Instantiations in Python
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The above section indicates that C++ types are generally registered with
Python, and a similar approach could be used for function and method templates.
However, these templates usually fit a certain pattern and can be Pythonized in
such a way that simplifies implementation, but may change the "feel" of the
signature.

Two common (non-metaprogramming) applications of templated functions and
methods present in Drake are `emplace <https://en.cppreference.com/w/cpp/container/vector/emplace>`_-like
functionality (using `parameter packs
<https://en.cppreference.com/w/cpp/language/parameter_pack>`_) and
`type erasure <https://en.wikipedia.org/wiki/Type_erasure>`_.
However, Python doesn't literally support these C++ language features. So, in
binding them, they get "Pythonized".

C++ APIs which use parameter packs, such as:

.. code-block:: cpp

    DiagramBuilder<T>::AddSystem<SystemType>(args...)
    MultibodyPlant<T>::AddJoint<JointType>(args...)
    MultibodyPlant<T>::AddFrame<FrameType>(args...)

will become the following in Python:

.. code-block:: pycon

    DiagramBuilder_[T].AddSystem(SystemType(args, ...))
    MultibodyPlant_[T].AddJoint(JointType(args, ...))
    MultibodyPlant_[T].AddFrame(FrameType(args, ...))

where the ``*Type`` tokens are replaced with the concrete type in question
(e.g. ``Adder_[T]``, ``RevoluteJoint_[T]``, ``FixedOffsetFrame_[T]``).

Similarly, type-erasure C++ APIs that look like:

.. code-block:: cpp

    InputPort<T>::Eval<ValueType>(context)
    GeometryProperties::AddProperty<ValueType>(group_name, name, value)

will become the following in Python:

.. code-block:: pycon

    InputPort_[T].Eval(context)
    GeometryProperties.AddProperty(group_name, name, value)

Debugging with the Python Bindings
----------------------------------

You may encounter issues with the Python Bindings that may arise from the
underlying C++ code, and it may not always be obvious what the root cause is.

The first step to debugging is to consider running your code using the
``trace`` module. It is best practice to always have a ``main()`` function, and
have a ``if __name__ == "__main__"`` clause. If you do this, then it is easy to
trace. As an example:

..
    N.B. These code snippets should be kept in sync with
    `drake_py_unittest_main.py`.

.. code-block:: python

    def reexecute_if_unbuffered():
        """Ensures that output is immediately flushed (e.g. for segfaults).
        ONLY use this at your entrypoint. Otherwise, you may have code be
        re-executed that will clutter your console."""
        import os
        import shlex
        import sys
        if os.environ.get("PYTHONUNBUFFERED") in (None, ""):
            os.environ["PYTHONUNBUFFERED"] = "1"
            argv = list(sys.argv)
            if argv[0] != sys.executable:
                argv.insert(0, sys.executable)
            cmd = " ".join([shlex.quote(arg) for arg in argv])
            sys.stdout.flush()
            os.execv(argv[0], argv)


    def traced(func, ignoredirs=None):
        """Decorates func such that its execution is traced, but filters out any
         Python code outside of the system prefix."""
        import functools
        import sys
        import trace
        if ignoredirs is None:
            ignoredirs = ["/usr", sys.prefix]
        tracer = trace.Trace(trace=1, count=0, ignoredirs=ignoredirs)

        @functools.wraps(func)
        def wrapped(*args, **kwargs):
            return tracer.runfunc(func, *args, **kwargs)

        return wrapped


    # NOTE: You don't have to trace all of your code. If you can identify a
    # single function, then you can just decorate it with this. If you're
    # decorating a class method, then be sure to declare these functions above
    # it.
    @traced
    def main():
        insert_awesome_code_here()


    if __name__ == "__main__":
        reexecute_if_unbuffered()
        main()



.. note::

    If you are developing in Drake and are using the ``drake_py_unittest``
    macro, you can specify the argument ``--trace=user`` to get the same
    behavior.

This generally should help you trace where the code is dying. However, if you
still need to dig in, you can build the bindings in debug mode, without symbol
stripping, so you can debug with ``gdb`` or ``lldb``:

.. code-block:: shell

    cmake -DCMAKE_BUILD_TYPE=Debug ../drake

.. warning::

    If you have SNOPT enabled (either ``-DWITH_SNOPT=ON`` or
    ``-DWITH_ROBOTLOCOMOTION_SNOPT=ON``), symbols will *still* be stripped.

For Developers
--------------

If you are developing Python bindings, please see the Doxygen page
`Python Bindings <https://drake.mit.edu/doxygen_cxx/group__python__bindings.html>`_ which provides information on programming conventions, documentation, tips
for debugging, and other advice.
