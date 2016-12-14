.. _directory_structure:

*******************
Directory Structure
*******************

The current directory structure in Drake is organized to (1) group functional
components, (2) limit dependencies, and (3) prevent circular dependencies.

.. _directory_structure_special_directories:

Special Directories
===================

All code that implements core functionality in Drake must adhere to the strict
code standards described in :ref:`Developer Notes <developer_notes>`, and must
have unit test coverage via code implemented in a ``test`` subdirectory
immediately below the core implementation.

To promote rapid development of research ideas within the code base,
we allow for some exceptions to this rule for experimental code.  This includes
any subdirectory labeled ``dev`` and directories with a special ``README.md``
file describing the protocol for that directory.  For PRs that affect only these
experimental directories, one feature review is sufficient; platform review is
not required.  If build or test targets in these directories break, and the PR
author or on-call build cop cannot trivially resolve them, a GitHub issue will
be assigned to the author of the code. If the issue is not resolved within 24
hours, the core development team may disable the offending targets.


.. _directory_structure_controlling_dependencies:

Controlling Dependencies
========================

We would like to avoid circular dependencies in the source code tree.  For
example, some code in ``drake/systems`` depends on code in ``drake/solvers``, so
code in ``drake/solvers`` should *not* depend on ``drake/systems``.

The ``drake/examples`` directories are logically after all of the core libraries
in ``drake``.  ``test`` directories located throughout the code are logically
last in the dependency tree -- they are allowed to depend on any of the other
directories.  For example, ``test`` code **is** allowed to use models/code from
the ``examples`` directories to test core functionality, as long as that
dependency is explicitly declared in the build system rules for that test.
