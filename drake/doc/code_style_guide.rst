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

Drake strictly follows the
`Google C++ Style Guide <https://google.github.io/styleguide/cppguide.html>`_
except for the specific clarifications, exceptions, and additional rules noted
below.

.. _code-style-guide-cpp-clarifications:

Clarifications
--------------

* Always prefer long, human-readable variable/method/class names to short
  acronyms.
* Manually provide user gradients only when we know more than AutoDiffScalar
  possibly could (e.g. sparsity of the gradients).
* For the `Exceptions
  <https://google.github.io/styleguide/cppguide.html#Exceptions>`_ style rule,
  we clarify as follows. Throwing exceptions is permitted and encouraged for
  error handling. Unit tests may catch exceptions using
  `EXPECT_THROW <https://github.com/google/googletest/blob/master/googletest/docs/AdvancedGuide.md#exception-assertions>`_
  if the exception is documented in the API. Otherwise, catching exceptions is
  forbidden. For more context, see
  `PR #3759 <https://github.com/robotlocomotion/drake/pull/3759>`_.

  * We allow exceptions to be thrown because it enables a more detailed
    description of the error to be provided relative to an assert statement.
  * **Note:** This is a work-in-progress rule, but captures our
    currently-in-effect style. We are open to discussion on additional uses for
    exceptions if and when the need arises.

* No dynamic allocation in the inner simulation/control loops.  Code should be
  still be thread-safe (e.g. be careful with pre-allocations).
* Classes and methods should be documented using
  `Doxygen <https://www.stack.nl/~dimitri/doxygen/manual/docblocks.html>`_.

  * Only use Doxygen comments (``///`` or ``/** */``) on published APIs (public
    or protected classes and methods).  Code with private access or declared in
    ``.cc`` files should not use the Doxygen format.
  * If you decide to use Doxygen formatting hints, then those *must* render
    correctly. For instructions on how to generate the Doxygen website, click
    :ref:`here <documentation-generation-instructions>`. For additional
    background information, see
    `PR #3584 <https://github.com/RobotLocomotion/drake/pull/3584>`_.
  * Prefer Doxygen comment blocks that are readable in both a rendered and
    un-rendered state. This could mean foregoing the most beautiful LaTeX
    formatting for some serviceable text equations readable in the code. Or, you
    may want to augment beautiful-but-unreadable formatting with a simplified
    presentation of the same information to accommodate future programmers, who
    are likely to only see the header file. For more background information, see
    `PR #3584 <https://github.com/RobotLocomotion/drake/pull/3584>`_.

* Embrace templates/C++14 when it makes the code more correct (more clear or
  more readable also implies more correct).  Minimize template requirements on
  public interfaces.  Avoid explicit template instantiations in cc files when
  possible.
* For the `order of includes
  <https://google.github.io/styleguide/cppguide.html#Names_and_Order_of_Includes>`_
  style rule, separate each category of ``#include`` statements with a blank
  line. Then, accept whatever
  :ref:`clang-format <code-style-tools-clang-format>` enforces.
* The `Function Names
  <https://google.github.io/styleguide/cppguide.html#Function_Names>`_
  rule specifies that the names of "very cheap" methods may be all lower-case
  with underscores between words. It defines "very cheap" as a method that you
  wouldn't hesitate calling from within a loop. We clarify that this method
  should have a time complexity of O(1) and be less than 5 lines long.
* The test file for the library declarations in ``drake/foo/bar.h`` should be
  ``drake/foo/test/bar_test.cc``.
  (`#2182 <https://github.com/RobotLocomotion/drake/issues/2182>`_)
* When using `Integer Types
  <https://google.github.io/styleguide/cppguide.html#Integer>`_
  within Drake, unsigned types are forbidden, with the following exceptions
  (per `#2514 <https://github.com/RobotLocomotion/drake/issues/2514>`_):

  * ``uint32_t`` or ``uint64_t`` are allowed for bitfields, and
  * ``size_t`` is allowed when a (1) non-Drake API uses unsigned types, and (2)
    casting to a signed type during Drake's interactions with the non-Drake API
    would obscure the readability of our code, and (3) subtraction underflow
    below zero is obviously not at risk.

* When using `Integer Types
  <https://google.github.io/styleguide/cppguide.html#Integer>`_
  within Drake, `ptrdiff_t` is forbidden, with the following exceptions
  (per `#2514 <https://github.com/RobotLocomotion/drake/issues/2514>`_):

  * ``ptrdiff_t`` is allowed when doing arithmetic on bare pointers (this is
    very rare).  Do not use it as a generic "large signed integer" type, nor
    as a generic "index into a matrix" type.

* For the `Use of const
  <https://google.github.io/styleguide/cppguide.html#Use_of_const>`_ style rule,
  we clarify that:

  * A class member variable *must* be declared ``const`` if it is not modified
    after the class is constructed, and
  * You *must not* use ``const`` in a function declaration where it adds no
    meaning. That occurs in pass-by-value parameter declarations, where
    ``const int i`` and ``int i`` mean the same thing, and in return-by-value
    declarations, where ``int f()`` and ``const int f()`` are also synonymous.
    You may add ``const`` to such parameter declarations in the function
    *definition*, where it does indicate that the implementation will not
    modify its own copy of the parameter value. The C++ standard explicitly
    states that the signatures are identical with or without the ``const`` in
    these cases, see `Overloadable declarations
    <http://www.lcdf.org/c%2B%2B/clause13.html>`_. (This applies to
    ``volatile`` also.)

    If you want to declare and define a function in one place, you have
    several options:

    * Forgo marking the parameters as ``const`` (not a great loss for short
      functions defined inline), or
    * create some ``const`` local variables initialized to the supplied
      parameter values (likely to be optimized away by the compiler), or
    * split the declaration and definition (be sure to add the ``inline``
      keyword if the function would otherwise have been implicitly inlined).

* For the `Pointer and Reference Expressions Rule <https://google.github.io/styleguide/cppguide.html#Pointer_and_Reference_Expressions>`_,
  we clarify as follows. When declaring a pointer or a reference, the "``*``"
  and "``&``" symbols must be next to the variable *type*, not the variable
  *name*. In other words use "``const MyClass& foo;``" instead of
  "``const MyClass &foo;``". This is what is enforced by :ref:`clang-format <code-style-tools-clang-format>`. For additional context, see
  `this comment thread <https://github.com/robotlocomotion/drake/pull/3830#issuecomment-254849776>`_.

.. _code-style-guide-cpp-exceptions:

Exceptions
----------

* Method names may violate Google standards and the "long, human-readable"
  standard above if a short, non-compliant name more closely matches the common
  conventions of the field.  For instance, the matrix portion of a linear
  complementarity constraint is traditionally 'M' (one letter, upper-case); it
  is not mandatory to downcase it or give it a more verbose name.
* No need for a copyright line at the top of every file (this will change soon,
  see: `issue #1805 <https://github.com/RobotLocomotion/drake/issues/1805>`_).
* While we encourage you to `Include What You Use (IWYU)
  <https://google.github.io/styleguide/cppguide.html#Names_and_Order_of_Includes>`_
  since it improves code transparency and readability, it will not be strictly
  enforced. Instead, we enforce a "weak include" style that simply requires
  every symbol referenced within a file be covered by the transitive closure of
  all `#include` statements in the file. We decided to make this exception
  because (1) we can always adopt an IWYU rule later, (2) to reduce verbosity,
  and (3) we don't have a tool to enforce IWYU at this time.
* We do not follow `The #define Guard
  <https://google.github.io/styleguide/cppguide.html#The__define_Guard>`_
  rule and instead use ``#pragma once``. The advantages of using ``#pragma
  once`` are (1) it does not need to be updated each time the name of the
  header file changes, and (2) it prevents silly mistakes that occur when a
  developer copy-pastes a header file and forgets to update its ``#define``
  guard. The known drawbacks of using ``#pragma once``, namely the possibility
  of lack of compiler support and compiler-dependent-behavior, is mitigated
  since Drake has a :ref:`limited set of officially supported platform
  configurations <supported-configurations>` on which correct behavior will be
  guaranteed.
  (`#2104 <https://github.com/RobotLocomotion/drake/issues/2104>`_)
* We have an exception to the `Implicit Conversions
  <https://google.github.io/styleguide/cppguide.html#Implicit_Conversions>`_
  rule.  For readability and consistency with external libraries like Eigen, we
  allow one-argument constructors without the `explicit` tag (implicit cast
  operators) if both types involved implement basic arithmetic operators
  commonly used in arithmetic exceptions (at minimum `+`, both unary and binary
  `-`, binary `*`, and `==`).  This lets us write concise mathematical
  expressions using math-like objects without a proliferation of
  `static_cast<>` statements.  (For context, see `#2231
  <https://github.com/RobotLocomotion/drake/pull/2231>`_)
* The `Self-contained headers
  <https://google.github.io/styleguide/cppguide.html#Self_contained_Headers>`_
  rule may be disobeyed when implementing the
  :ref:`C++ *-inl.h files <cxx-inl-files>` pattern in its exact form.

.. _code-style-guide-cpp-addon-rules:

Additional Rules
----------------
* Use the ``GTEST`` prefix in unit test declarations. For instance, use
  ``GTEST_TEST(Group, Name)`` instead of ``TEST(Group, Name)``.
  (`#2181 <https://github.com/RobotLocomotion/drake/issues/2181>`_)
* Always use `in-class member initialization
  <http://www.stroustrup.com/C++11FAQ.html#member-init>`_ for built-in data
  types that would would otherwise be uninitialized, including numerical
  types, pointers, and enumerations. The syntax ``int count_{};`` (called
  `value initialization
  <http://en.cppreference.com/w/cpp/language/value_initialization>`_)
  ensures that these types are zero-initialized rather than left with
  unpredictable content (informally known as "garbage"). You may also
  provide explicit values such as ``bool flag_{false};`` (for clarity) or
  ``bool uninitialized_{true};`` (because zero is the wrong initial value).
  You should always provide a value for ``enum`` members since zero might
  not be one of the allowed enumerations. Class objects are responsible
  for their own construction so they do not need to be member-initialized,
  but you can do so if the default constructor does not provide the
  behavior you want. Note that fixed-size Eigen objects are intentionally
  left uninitialized; if you want yours zero-initialized you can
  member-initialize it by passing an appropriate ``Zero``, for example:
  ``Eigen::Matrix3d mat_{Eigen::Matrix3d::Zero()};``.
* After including ``<cstddef>``, assume that ``size_t``
  is defined in the global namespace. Do not preface it with ``std::``
  and do not write ``using std::size_t`` in
  your code. There is a hypothetical possibility that this won't work on
  some compiler someday but we deem the risk acceptable in trade for
  allowing this common, clutter-reducing practice. For discussion, see
  `stackoverflow <http://stackoverflow.com/questions/5813700/difference-between-size-t-and-stdsize-t>`_
  and Drake `#2374 <https://github.com/RobotLocomotion/drake/issues/2374>`_.
* Rules for assertions:

  * Never use ``assert()`` from ``<cassert>``.
  * Use ``DRAKE_ASSERT(`` *condition* ``)`` to compile and assert only
    in debug builds.
  * Use ``DRAKE_DEMAND(`` *condition* ``)`` to assert in any kind of
    build (including release).
  * When checking for nonnullness, either ``DRAKE_DEMAND(ptr)`` or
    ``DRAKE_DEMAND(ptr != nullptr)`` is allowed; use whichever seems
    clearer in context.
  * When failing unconditionally, never use ``DRAKE_ASSERT(false)``;
    instead, use ``DRAKE_ABORT()``.
  * For discussion, see Drake
    `#1935 <https://github.com/RobotLocomotion/drake/issues/1935>`_ and
    `#3355 <https://github.com/RobotLocomotion/drake/issues/3355>`_.

* The ``main()`` method should be as brief as possible since it exists outside
  of namespace ``drake``. It should simply call another method that is
  appropriately namespaced within namespace ``drake``. The method can be called
  "``main()``" since it is allowed by the style guide's
  `exceptions to naming rules <https://google.github.io/styleguide/cppguide.html#Exceptions_to_Naming_Rules>`_, though other method names like
  "``exec()``" are also acceptable.

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
* The `codeCheck` utility will run `mlint` on the code with appropriate
  warnings disabled.  Eventually, the code should pass this check (but we're
  still far from it)

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
within the example's directory. While a full ``package.xml`` file that contains
every `required field <http://wiki.ros.org/catkin/package.xml#Required_Tags>`_
would be ideal, for Drake's purposes, the following minimal ``package.xml`` file
is sufficient::

    <!--
    This XML file is used by:
      drake-distro/drake/systems/plants/xmlUtil.cpp
    Method:
      searchDirectory()
    -->
    
    <package format="2">
      <name>package_name</name>
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
