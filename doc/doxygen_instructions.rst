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
simple diagram documenting their input and output ports using the macro:

.. code-block:: none

  @system{ system name/label,
           @input_port{input port 1 name} @input_port{input port 2 name},
           @output_port{output port 1 name} @output_port{output port 2 name}}

The first argument is the contents of the html table cell inside the box. The
recommendation is to use (at least) the system name, but you may also get
creative and have images, etc, too.  Note that the comma delimiter separates the
input and output port listings (there is no comma between ports).  Where
possible, the names here should match the names given to the ports in the code,
so that error messages, debugging, and automatically-generated System diagrams
match the documentation.

Some systems have variable numbers of ports, or ports that are created
conditionally.  Prefer to add all possible ports to the diagram; see the `Adder
<https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_adder.html>`_ system
for an example of a system with a variable number of ports.  The ``@system``
doxygen tag can be used multiple times to add multiple renders of the system to
the documentation, if the system is most clearly described by a few examples.

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
