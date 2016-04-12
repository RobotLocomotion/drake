.. _code-style-guide:

****************
Code Style Guide
****************

This section defines a style guide which should be followed by all code that is written
in Drake. Being consistent with this style will make the code easier to read, debug,
and maintain. To ensure your code is style compliant, consider using
:ref:`tools for complying with coding style <code-style-tools>`.

Note: Many of the files in the repository were written before this style guide, or did
not follow it precisely.  If you find style errors, go ahead and change it and submit
a pull request.

.. contents:: `Table of contents`
   :depth: 3
   :local:

C++ Style
=========

Drake strictly follows the `Google C++ Style Guide <https://google.github.io/styleguide/cppguide.html>`_  except for the specific clarifications and exceptions noted below.

Clarifications
--------------

* Always prefer long, human-readable variable/method/class names to short acronyms.
* Manually provide user gradients only when we know more than AutoDiffScalar possibly could (e.g. sparsity of the gradients).
* Use exceptions for error handling.  Essential control loops must be exception safe.
* No dynamic allocation in the inner simulation/control loops.  Code should be still be thread-safe (e.g. be careful with pre-allocations).
* Classes and methods should be documented using `doxygen <https://www.stack.nl/~dimitri/doxygen/manual/docblocks.html>`_.
* Embrace templates/C++11 when it makes the code more correct (more clear or more readable also implies more correct).  Minimize template requirements on public interfaces.  Avoid explicit template instantiations in cc files when possible.
* For the `order of includes <https://google.github.io/styleguide/cppguide.html#Names_and_Order_of_Includes>`_ style rule, separate each category of ``#include`` statements with a blank line. Then, accept whatever :ref:`clang-format <code-style-tools-clang-format>` enforces.


Exceptions
----------

* Method names may violate Google standards and the "long, human-readable" standard above if a short, non-compliant name more closely matches the common conventions of the field.  For instance, the matrix portion of a linear complementarity constraint is traditionally 'M' (one letter, upper-case); it is not mandatory to downcase it or give it a more verbose name.
* No need for a copyright line at the top of every file (this will change soon, see: `issue #1805 <https://github.com/RobotLocomotion/drake/issues/1805>`_).
* While we encourage you to `Include What You Use (IWYU) <https://google.github.io/styleguide/cppguide.html#Names_and_Order_of_Includes>`_ since it improves code transparency and readability, it will not be strictly enforced. Instead, we enforce a "weak include" style that simply requires every symbol referenced within a file be covered by the transitive closure of all `#include` statements in the file. We decided to make this exception because (1) we can always adopt an IWYU rule later, (2) to reduce verbosity, and (3) we don't have a tool to enforce IWYU at this time.


MATLAB Style
============

* All of the above rules still hold as relevant (e.g. variable names).
* A short list of variable name exceptions for common acronyms:
   * `rpy` or `somethingRPY` (for roll-pitch-yaw)
* All classes and methods should be commented with doxygen compatible formatting (using the tags `@param` to describe each input, `@option` to describe the elements of an option structure, `@retval` to describe each output, and `@default` to describe default values for an input.  Class methods need not document the trivial first input argument (which is the class object) with a `@param` tag.
* Calls to MATLAB class member functions in speed critical loops for classes which overload subsref use `memberFunc(obj,...)` instead of `obj.memberFunc(...)`.  This is because obj.member calls the `subsref` method, which is only notably slower for classes which have overloaded `subsref`.  All other calls should use `obj.memberFunc(...)`.
* All methods that are outside runtime execution loops begin by checking their inputs (e.g. with `typecheck`,`sizecheck`,`rangecheck`,etc).  Methods that get called repeatedly inside a simulation or optimization loop should not perform these checks.
* All methods (including mex) should treat `nargout==0` as if we received `nargout==1`
* The `codeCheck` utility will run `mlint` on the code with appropriate warnings disabled.  Eventually, the code should pass this check (but we're still far from it)


Java Style
==========

We also strictly follow the `Google Java Style Guide` <https://google.github.io/styleguide/javaguide.html>`_ .  Here are some additional comments:

* Every class and method should have a brief `_javadoc_` associated with it.
* All Java classes should be in packages relative to the Drake root,
   e.g.: package drake.examples.Pendulum


LCM Style
=========

* LCM types are under_scored with a leading `lcmt_` added. If the type is specific to a particular robot, then it begins with `lcmt_robotname_`.
* Variable names in LCM types follow the rules above.


Shell Script Style
==================

We follow the `Google Shell Style Guide` <https://google.github.io/styleguide/shell.xml>`_.


Version numbers
===============

We'll adopt the following convention for version numbers in Drake:  The version number will have the format W.X.Y.Z where

* W = major release number
* X = minor release number
* Y = development stage*
* Z = build

Development stage is one of four values:
* 0 = alpha (buggy, not for use)
* 1 = beta (mostly bug-free, needs more testing)
* 2 = release candidate (rc) (stable)
* 3 = release

Z (build) is optional. This is probably not needed but could just refer to the revision of
the repository at the time of snapshot. Numbered versions should be referenced via tags.
