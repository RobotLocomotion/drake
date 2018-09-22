.. _doxygen-instructions:

********************
Doxygen Instructions
********************

This section contains instructions on how to use
`Doxygen <http://www.stack.nl/~dimitri/doxygen/>`_ with Drake.

.. _doxygen-style-guide:

Doxygen Style Guide
===================

Coming soon. See issue
`#2051 <https://github.com/RobotLocomotion/drake/issues/2051>`_ and PR
`#2359 <https://github.com/RobotLocomotion/drake/pull/2359>`_.

Classes that implement the `drake::systems::System <http://drake.mit
.edu/doxygen_cxx/classdrake_1_1systems_1_1_system.html>`_ interface should add a
simple diagram documenting their input and output ports using the macro:

.. code-block:: python

  @system{ system name,
           @input_port{input port 1 name} @input_port{input port 2 name},
           @output_port{output port 1 name} @output_port{output port 2 name}}

Note that the comma delimiter separates the input and output port listings
(there is no comma between ports).

Some systems have variable numbers of ports, or ports that are created
conditionally.  Prefer to add all possible ports to the diagram; see the
`Adder <http://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_adder.html>`_
system for an example of a system with a variable number of ports.  The
`@system` doxygen tag can be used multiple times to add multiple renders of
the system to the documentation, if the system is most clearly described by a
few examples.

Documentation Tips and Tricks
=============================

We encourage the use of `unicode <unicode_tips_tricks>`_ in documentation.

For documenting block diagrams, consider using ascii art.  See, for instance,

- http://asciiflow.com/


.. _doxygen-generation:

Doxygen Website Generation
==========================

To generate and view Drake's Doxygen website, see:
:ref:`Documentation Instructions<documentation-generation-instructions>`.
