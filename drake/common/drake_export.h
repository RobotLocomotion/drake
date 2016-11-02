#pragma once

#ifdef DRAKE_EXPORT
#error DRAKE_EXPORT should not be pre-defined
#endif

/// A macro that marks symbols as exported with default visibility.
#define DRAKE_EXPORT __attribute__((visibility("default")))
