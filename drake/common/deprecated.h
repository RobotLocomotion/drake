#pragma once

/// @file
/// Provides a macro for marking a method as being deprecated. It works with
/// C++14, VC++, gcc, and clang.

#ifdef DRAKE_DOXYGEN_CXX

/// @p DEPRECATED_14(MSG) is a macro that applies a deprecate attribute to a
/// method. If the method is called, a warning will be printed at compile-time.
/// A macro is needed since there is no standard way to do this across different
/// compilers.
#define DEPRECATED_14(MSG)

#else  // DRAKE_DOXYGEN_CXX

#ifndef SWIG
  #if __cplusplus >= 201402L
    /* C++14 */
    #define DEPRECATED_14(MSG) [[deprecated(MSG)]]
  #elif _MSC_VER
    /* VC++ just says warning C4996 so add "DEPRECATED" to the message. */
    #define DEPRECATED_14(MSG) __declspec(deprecated("DEPRECATED: " MSG))
  #else /* gcc or clang */
    #define DEPRECATED_14(MSG) __attribute__((deprecated(MSG)))
  #endif
#else  // SWIG
  #define DEPRECATED_14(MSG)
#endif  // SWIG

#endif  // DRAKE_DOXYGEN_CXX
