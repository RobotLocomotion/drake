#pragma once

/** @file
Provides a portable macro for use in generating compile-time warnings for
use of code that is permitted but discouraged. **/

#ifdef DRAKE_DOXYGEN_CXX
/** Use `DRAKE_DEPRECATED("message")` to discourage use of particular
classes, typedefs, variables, non-static data members, functions, arguments,
enumerations, and template specializations. A compile time warning will be
issued displaying the given message, preceded by "DRAKE DEPRECATED: ".

This is typically used for constructs that have been replaced by something
better and it is good practice to suggest the appropriate replacement in the
deprecation message. Deprecation warnings are conventionally used to convey to
users that a feature they are depending on will be removed in a future release.

Usage: @code
  // Attribute comes *before* declaration of a deprecated function or variable;
  // no semicolon is allowed.
  DRAKE_DEPRECATED("f() is slow; use g() instead.")
  int f(int arg);

  // Attribute comes *after* struct, class, enum keyword.
  class DRAKE_DEPRECATED("Use MyNewClass instead.")
  MyClass {
  };
@endcode

This feature is standard in C++14 compilers via the `[[deprecated]]` attribute,
and this macro will generate the standard attribute when compiled with a
C++14-compliant compiler. **/
#define DRAKE_DEPRECATED(message)

#else  // DRAKE_DOXYGEN_CXX

/* C++14 introduces a standard way to mark deprecated declarations. Before
that we can use non-standard compiler hacks. */
#ifndef SWIG
  /* Figure out the best form of deprecation for this compiler. */
  #if __cplusplus >= 201402L
    /* C++14 */
    #define DRAKE_DEPRECATED(MSG) [[deprecated("\nDRAKE DEPRECATED: " MSG)]]
  #else /* gcc or clang */
    #define DRAKE_DEPRECATED(MSG) \
      __attribute__((deprecated("\nDRAKE DEPRECATED: " MSG)))
  #endif
#else /* Swigging */
  #define DRAKE_DEPRECATED(MSG)
#endif

#endif  // DRAKE_DOXYGEN_CXX
