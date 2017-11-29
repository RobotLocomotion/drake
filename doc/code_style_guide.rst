.. _code-style-guide:

****************
Code Style Guide
****************

This section defines a style guide which should be followed by all code that is
written in Drake. Being consistent with this style will make the code easier to
read, debug, and maintain. To ensure your code is style compliant, consider
using :ref:`tools for complying with coding style <code-style-tools>`.

See also the brief
:ref:`Code Review Checklist <code-review-checklist>`,
where a list of the most frequent problems are collected.

Note: Many of the files in the repository were written before this style guide,
or did not follow it precisely.  If you find style errors, go ahead and change
it and submit a pull request.

.. contents:: `Table of contents`
   :depth: 3
   :local:

.. _code-style-guide-cpp:

C++ Style
=========

The Drake C++ style guide (which is derived from the Google C++ style guide)
can be found
`on its own page <http://drake.mit.edu/styleguide/cppguide.html>`_.
Its source lives in
`the styleguide repository <https://github.com/RobotLocomotion/styleguide>`_.

.. _code-style-guide-python:

Python Style
============

Drake Python code uses Python 2.7.

Drake strictly follows `PEP 8 -- Style Guide for Python Code
<https://www.python.org/dev/peps/pep-0008/>`_ except for the specific
clarifications, exceptions, and additional rules noted below. Since PEP 8
incorporates `PEP 257 -- Docstring Conventions
<https://www.python.org/dev/peps/pep-0257/>`_, Drake follows its
recommendations as well.

In addition, Drake recommends use of ``pylint`` for automatic error
checking. See :ref:`tools for complying with coding style <code-style-tools>`
for full guidance.

Sadly ``pylint`` does not check all the conditions enumerated by
PEP 8. Therefore, Drake recommends the use of ``pep8.py`` as well. See
:ref:`tools for complying with coding style <code-style-tools>` for details.`

.. _code-style-guide-python-clarifications:

Clarifications
--------------

* External, third-party, and auto-generated source files are not to be checked
  for style.
* Always prefer long, human-readable variable/method/class names to short
  acronyms.

.. _code-style-guide-python-exceptions:

Exceptions
----------

* Lines containing a long URL may be longer than 80 columns if necessary to
  avoid splitting the URL.

.. _code-style-guide-python-addon-rules:

Additional Rules
----------------

* When importing in-tree modules, always use absolute import paths; explicit
  relative import paths are disallowed. See the `PEP 8 discussion of imports
  <https://www.python.org/dev/peps/pep-0008/#imports>`_ for more detail.
* Sometimes ``__init__.py`` files are necessary, for Python's ``import``
  mechanism; these files should be non-empty (via a copyright notice, for
  example). Rationale: 0-byte files can be mistakenly perceived as the result
  of some error or accident.
* When using the ``logging`` module, avoid its lazy-formatting
  syntax. Rationale: exceptions raised in lazy formatting get printed to
  ``stderr``, but are otherwise ignored, and thus may escape notice.
* Executable files should use the following "shebang" line::

    #!/usr/bin/env python

  Rationale: ``/usr/bin/env`` enables a ``PATH`` search for python. On macOS
  systems configured for Drake, this gives a better result than
  ``/usr/bin/python``.

.. _code-style-guide-matlab:

MATLAB Style
============

* All of the above rules still hold as relevant (e.g. variable names).
* A short list of variable name exceptions for common acronyms:
   * `rpy` or `somethingRPY` (for roll-pitch-yaw)
* All classes and methods should be commented with Doxygen compatible
  formatting (using the tags `@param` to describe each input, `@option` to
  describe the elements of an option structure, `@retval` to describe each
  output, and `@default` to describe default values for an input.  Class
  methods need not document the trivial first input argument (which is the
  class object) with a `@param` tag.
* Calls to MATLAB class member functions in speed critical loops for classes
  which overload subsref use `memberFunc(obj,...)` instead of
  `obj.memberFunc(...)`.  This is because obj.member calls the `subsref`
  method, which is only notably slower for classes which have overloaded
  `subsref`.  All other calls should use `obj.memberFunc(...)`.
* All methods that are outside runtime execution loops begin by checking their
  inputs (e.g. with `typecheck`,`sizecheck`,`rangecheck`,etc).  Methods that
  get called repeatedly inside a simulation or optimization loop should not
  perform these checks.
* All methods (including mex) should treat `nargout==0` as if we received
  `nargout==1`

.. _code-style-guide-java:

Java Style
==========

We also strictly follow the `Google Java Style Guide
<https://google.github.io/styleguide/javaguide.html>`_.
Here are some additional comments:

* Every class and method should have a brief `_javadoc_` associated with it.
* All Java classes should be in packages relative to the Drake root,
   e.g.: package drake.examples.Pendulum

.. _code-style-guide-lcm:

LCM Style
=========

* LCM types are under_scored with a leading `lcmt_` added. If the type is
  specific to a particular robot, then it begins with `lcmt_robotname_`.
* Variable names in LCM types follow the rules above.

.. _code-style-guide-package-xml:

package.xml Style
=================

`Robot Operating System (ROS) <http://www.ros.org/>`_ organizes code and data
into packages. Each package is located in its own directory, which contains a
file called ``package.xml``. The official instructions on what this file should
contain is given `here <http://wiki.ros.org/catkin/package.xml>`_. Drake uses
this same file for defining and finding packages. Specifically, it is searched
for by
`populatePackageMap() <https://github.com/RobotLocomotion/drake/blob/7bbcb0728a06c0abdd695fd8a5db1879bb5354bb/drake/systems/plants/xmlUtil.h#L160>`_.
Note that ``package.xml`` files are necessary even if you're *not* using Drake
with ROS because the model files used by Drake (e.g., URDF and SDF), frequently
refer to resources like mesh files via a ``package://`` syntax, whose full paths
are resolved by the ``package.xml`` files.

When adding a model to Drake
(typically in `drake-distro/drake/examples/ <https://github.com/RobotLocomotion/drake/tree/master/drake/examples>`_),
you will need to add a ``package.xml`` file to the example's directory to enable
modeling files like URDF and SDF to refer to resources like mesh files contained
within the example's directory. Please ensure that your ``package.xml`` file
contains every
`required field <http://wiki.ros.org/catkin/package.xml#Required_Tags>`_.
The following minimal ``package.xml`` file can get you started::

    <!--
    This XML file is used by:
      drake-distro/drake/systems/plants/xmlUtil.cpp
    Method:
      searchDirectory()
    -->

    <package format="2">
      <name>package_name</name>
      <version>0.0.0</version>
      <description>
        A description of your package.
      </description>
      <maintainer email="drake-users@mit.edu">Drake Users</maintainer>
      <license>BSD</license>
    </package>

In the above example, replace "package_name" with the name of your package. This
is typically the name of the directory holding the ``package.xml`` file.

.. _code-style-guide-shell-script:

Shell Script Style
==================

We follow the `Google Shell Style Guide
<https://google.github.io/styleguide/shell.xml>`_.

.. _code-style-guide-version-numbers:

Version numbers
===============

We'll adopt the following convention for version numbers in Drake:
The version number will have the format W.X.Y.Z where

* W = major release number
* X = minor release number
* Y = development stage*
* Z = build

Development stage is one of four values:

* 0 = alpha (buggy, not for use)
* 1 = beta (mostly bug-free, needs more testing)
* 2 = release candidate (rc) (stable)
* 3 = release

Z (build) is optional. This is probably not needed but could just refer to the
revision of the repository at the time of snapshot. Numbered versions should be
referenced via tags.

Supplementary documents
=======================

.. toctree::
  :maxdepth: 1

  cxx_inl
