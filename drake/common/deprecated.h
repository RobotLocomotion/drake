#pragma once

/// @file
/// Provides a macro for marking a method as being deprecated. It works with
/// C++14, VC++, gcc, and clang.

#ifdef DRAKE_DOXYGEN_CXX

/// @p DEPRECATED_14(MSG) is a macro that can be applied to a method. If the
/// method is called, a warning will be printed during compile-time. A macro
/// is needed since there is no single standard way to do this across different
/// compilers.
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
#else /* Swigging */
  #define DEPRECATED_14(MSG)
#endif

#endif  // DRAKE_DOXYGEN_CXX
