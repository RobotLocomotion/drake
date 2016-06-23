#pragma once

/** @file
Provides a portable macro for use in generating compile-time warnings for
use of code that is permitted but discouraged. **/

#ifdef DRAKE_DOXYGEN_CXX
/** `DRAKE_DEPRECATED("message")` should be used to discourage use of particular
classes, typedefs, variables, non-static data members, functions, enumerations,
and template specializations. A compile time warning will be issued displaying
the given message.

This is typically used for constructs that have been replaced by something
better and it is good practice to suggest the appropriate replacement in the
deprecation message. Usage:

@code
// Atribute comes before declaration of a deprecated function or variable.
DRAKE_DEPRECATED("f() is slow; use g() instead")
int f(int arg);

// Attribute comes after struct, class, enum keyword; no semicolon allowed.
class DRAKE_DEPRECATED("use MyNewClass instead")
MyClass {
};
@endcode

This feature is a standard attribute in C++14 compilers and this macro will
generate the standard feature when compiled with a C++14-compliant compiler. **/
#define DRAKE_DEPRECATED(message)

#else  // DRAKE_DOXYGEN_CXX

/* C++14 introduces a standard way to mark deprecated declarations. Before
that we can use non-standard compiler hacks. */
#ifndef SWIG
  /* Make sure warnings are enabled in VC++; it is a common Windows hack for
  programmers to turn them off due to much inconvenient deprecation of useful
  things by Microsoft. */
  #if _MSC_VER
    #pragma warning(default:4996)
  #endif
  /* Figure out the best form of deprecation for this compiler. */
  #if __cplusplus >= 201402L
    /* C++14 */
    #define DRAKE_DEPRECATED(MSG) [[deprecated(MSG)]]
  #elif _MSC_VER
    /* VC++ just says warning C4996 so add "DEPRECATED" to the message. */
    #define DRAKE_DEPRECATED(MSG) __declspec(deprecated("DEPRECATED: " MSG))
  #else /* gcc or clang */
    #define DRAKE_DEPRECATED(MSG) __attribute__((deprecated(MSG)))
  #endif
#else /* Swigging */
  #define DRAKE_DEPRECATED(MSG)
#endif

#endif  // DRAKE_DOXYGEN_CXX
