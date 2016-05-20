.. _cxx-inl-files:

*******************
C++ `*-inl.h` files
*******************

This section explains the "what, why, and how" of Drake's use of the
`*-inl.h` pattern for templated C++ code.

.. contents:: `Table of contents`
   :depth: 3
   :local:

Motivation
==========

Compiling templated code can take a long time, but in many cases only
a small set of concrete types are ever instantiated.  By moving
template method bodies out of the header file and using explicit
template instantiation, we can drastically speed up compile times.

The conventional approach to explicit template instantiation uses only
an ``*.h`` file and ``*.cc`` file.  The declarations go in the
``*.h``, and the definitions and instantiations go in the ``*.cc``.
This two-file solution is straightforward and well-known, so if it
meets your needs you should prefer it to the ``*-inl.h`` pattern shown
below.

The case for ``*-inl.h`` files appears when we wish to provide the
opportunity for calling code to use template types other than those
the ones instantiated in our ``*.cc`` file.  The problem is that
method bodies only exist in the ``*.cc`` file, so there is no
mechanism for calling code to instantiate them with different types.

The ``*-inl.h`` pattern works around this problem by placing the
method bodies in a third file, named ``*-inl.h``.  Code that only uses
standard types can includes the ``*.h`` file like normal.  Code that
requires a non-standard type instead includes the ``*-inl.h`` file.


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
  /// this class, please refer to @see http://drake.mit.edu/cxx_inl.html.
  ///
  /// Instantiated templates for the following kinds of T's are provided:
  /// - double
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
  /// Template method implementations for @see my_class.h.
  /// Most users should only include that file, not this one.
  /// For background, @see http://drake.mit.edu/cxx_inl.html.

  #include "my_class.h"

  template <typename T>
  T MyClass::get_random_number() const {
    return static_cast<T>(4.0);  // chosen by fair dice roll
  }

``my_class.cc``
---------------

The cc explicitly instantiates the templates (asks the compiler to
emit object code for some pre-defined types)::

  #include "my_class-inl.h"

  template MyClass<double>;

``main.cc``
-----------

Calling code uses the header, not the inl::

  #include "my_class.h"

  int main() {
    MyClass<double> die;
    std::cerr << die.get_random_number() << std::endl;
  }


Rules
=====

1. The file names must be ``.h`` and ``-inl.h`` and ``.cc``.
2. The comments in each of the two header files must cite this page.
3. The class must document which types are pre-instantiated.
