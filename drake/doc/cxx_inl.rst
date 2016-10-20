.. _cxx-inl-files:

*******************
C++ `*-inl.h` files
*******************

This section explains the "what, why, and how" of Drake's use of the
`*-inl.h` pattern for templated C++ code.  (The word `inl` here is an
abbreviation for `inline`.)

.. contents:: `Table of contents`
   :depth: 3
   :local:

Explicit template instantiation
===============================

Compiling templated code can take a long time, but in many cases only
a small set of concrete types are ever instantiated.  By moving
template method bodies out of the header file and using
`explicit template instantiation
<http://en.cppreference.com/w/cpp/language/class_template#Explicit_instantiation>`_,
we can drastically speed up compile times.

(Note that *implicit* template instantiation is still allowed, and
indeed encouraged when appropriate; this document only covers the
rules for *explicit* instantiation.)

Drake uses two approaches for explicit template instantiation; each
one is described immediately below.  We prefer the first "traditional"
approach when possible, because it is simpler.  The second "`*-.inl`
pattern" approach may be used when required.

Two-file, traditional approach
------------------------------

The traditional approach to explicit template instantiation uses only
an ``*.h`` file and ``*.cc`` file.  The declarations go in the
``*.h`` file, and their definitions and instantiations go in the ``*.cc`` file.

This two-file solution is straightforward and well-known, so if it
meets your needs you should prefer it.

Three-file, `-inl.h` pattern approach
-------------------------------------

The case for ``*-inl.h`` files appears when we wish to provide the
opportunity for calling code to use template types other than
the ones instantiated in our ``*.cc`` file.
The problem with the traditional approach is that
method bodies only exist in the ``*.cc`` file, so there is no
mechanism for calling code to instantiate them with different types.

The ``*-inl.h`` pattern works around this problem by placing the
method bodies in a third file named ``*-inl.h``.  Code that only uses
standard types should only include the ``*.h`` file.  Code that
requires a non-standard type should instead include only the ``*-inl.h`` file.


Example of the `*-inl.h` pattern
================================

Here is a minimal example.

``my_class.h``
--------------

The header file declares and documents the class::

  #pragma once

  /// MyClass ... (write an overview) ...
  ///
  /// @tparam T must be ... (document the requirements) ...
  ///
  /// This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
  /// this class, please refer to http://drake.mit.edu/cxx_inl.html.
  ///
  /// Instantiated templates for the following kinds of T's are provided:
  /// - double
  ///
  /// They are already available to link against in libmylibrary.
  template <typename T>
  class MyClass {
   public:
    T get_random_number() const;
  };

``my_class-inl.h``
------------------

The inl file defines the templated methods::

  #pragma once

  /// @file
  /// Template method implementations for my_class.h.
  /// Most users should only include that file, not this one.
  /// For background, see http://drake.mit.edu/cxx_inl.html.

  #include "my_class.h"

  template <typename T>
  T MyClass::get_random_number() const {
    return static_cast<T>(4.0);  // chosen by fair dice roll
  }

``my_class.cc``
---------------

The implementation file explicitly instantiates the templates
(it instructs the compiler to emit object code for some pre-defined types)::

  #include "my_class-inl.h"

  template class MyClass<double>;

``main.cc``
-----------

Calling code uses the `*.h` header, not the `*-inl.h` header::

  #include "my_class.h"

  int main() {
    MyClass<double> dice;
    std::cerr << dice.get_random_number() << std::endl;
  }

This works for all of the supported types listed in the `my_class.h` header.

``non-standard_main.cc``
------------------------

Unusual calling code uses the `*-inl.h` header, not the `*.h` header::

  #include "functional_form.h"
  #include "my_class-inl.h"

  int main() {
    MyClass<FunctionalForm> dice;
    std::cerr << dice.get_random_number() << std::endl;
  }

This is used for types not already listed in the `my_class.h` header.
This performs *implicit* instantiation (so is often slower to compile).
Whenever possible, prefer to add more explicitly-supported types to
`MyClass`'s API for everyone to use, rather than use this implicit form.


Rules
=====

1. The file names must end with ``.h`` and ``-inl.h`` and ``.cc``.
2. The comments in each of the two header files must cite this page.
3. The class must document which types are pre-instantiated.
