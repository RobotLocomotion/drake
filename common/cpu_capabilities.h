#pragma once

#include <cstdint>

namespace drake {

/** @addtogroup environment_variables
@{
@defgroup drake_disable_cpu_features DRAKE_DISABLE_CPU_FEATURES

Certain low-level code in Drake will scan for available CPU features at runtime
and dispatch to CPU-optimized implementations.

This offers the best performance, but can lead to different results when running
on different hardware.  Users can set the environment variable
`DRAKE_DISABLE_CPU_FEATURES` to a comma-separated list of CPU features to not
dispatch to at runtime. This is the same idea as <a
href="https://numpy.org/devdocs/reference/simd/build-options.html#runtime-dispatch">NumPy's
runtime dispatch</a>.  Drake first checks if `DRAKE_DISABLE_CPU_FEATURES` is set
and if not then falls back to check `NPY_DISABLE_CPU_FEATURES` instead, so that
users only need to set one variable.

If both `DRAKE_DISABLE_CPU_FEATURES` and `NPY_DISABLE_CPU_FEATURES` are unset,
then all runtime dispach features are enabled (maximum performance). If
`DRAKE_DISABLE_CPU_FEATURES` is set to the empty string, then all runtime
dispach features are enabled (maximum performance). This is true even when
`NPY_DISABLE_CPU_FEATURES` is set. The Drake option always has precedence.

For the full list of feature strings which can be disabled, refer to the <a
href="https://numpy.org/devdocs/reference/simd/build-options.html#supported-features">NumPy
list of features</a>.

For example, to disallow 512-bit Intel SIMD extensions:
@code{.sh}
export DRAKE_DISABLE_CPU_FEATURES=AVX512F
@endcode
@} */

namespace internal {

/* ... */
int64_t GetHighwayAllowedTargetMask();

}  // namespace internal
}  // namespace drake
