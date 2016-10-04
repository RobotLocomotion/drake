#pragma once

/// @file
/// Provide `DRAKE_EXPORT` macro to mark symbols as exported with default
/// visibility.

#ifdef DRAKE_EXPORT
#error DRAKE_EXPORT should not be pre-defined
#endif

#if defined(WIN32) || defined(WIN64)
#define DRAKE_EXPORT __declspec(dllexport)
#else
#define DRAKE_EXPORT __attribute__((visibility("default")))
#endif
