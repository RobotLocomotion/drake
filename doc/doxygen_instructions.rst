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

Classes that implement the `drake::systems::System <https://drake.mit
.edu/doxygen_cxx/classdrake_1_1systems_1_1_system.html>`_ interface should add a
simple diagram documenting their input and output ports using the custom doxygen-style ``@system`` and ``@endsystem`` tags:

.. code-block:: none

  @system
  name: Alchemist
  input_ports:
  - lead
  - more lead
  output_ports: 
  - gold
  @endsystem

The text between these tags is parsed as YAML, and must define the key ``name``
and may optionally define the lists ``input_ports`` and/or ``output_ports``. The
values may include ``html`` code.  The class name appears in the inside of the
rendered system block, you can get creative and add an image, or you can adjust
the text colors of your ports.

Some systems have variable numbers of ports, or ports that are created
conditionally.  Prefer to add all possible ports to the diagram; see the `Adder
<https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_adder.html>`_
system for an example of a system with a variable number of ports.  The
``@system/@endsystem`` tags can be used multiple times to add multiple renders
of the system to the documentation, if the system is most clearly described by a
few examples.

Documentation Tips and Tricks
=============================

We encourage the use of `unicode <unicode_tips_tricks>`_ in documentation.

For documenting block diagrams, consider using ascii art.  See, for instance,

- http://asciiflow.com/

The Emacs `ascii-art-to-unicode
<https://elpa.gnu.org/packages/ascii-art-to-unicode.html>`_ package can make
ascii-art block diagrams even prettier by replacing ``-``, ``|``, etc. with unicode
`box-drawing characters <https://en.wikipedia.org/wiki/Box-drawing_character>`_.


.. _doxygen-generation:

Doxygen Website Generation
==========================

To generate and view Drake's Doxygen website, see:
:ref:`Documentation Instructions<documentation-generation-instructions>`.
