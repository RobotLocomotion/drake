#pragma once

#include <cstdint>

namespace drake {

/** @defgroup drake_disable_cpu_features DRAKE_DISABLE_CPU_FEATURES
@ingroup environment_variables
@{

Certain low-level code in Drake will scan for available CPU features at runtime
and dispatch to CPU-optimized implementations.

This offers the best performance, but can lead to different results when running
on different hardware.  Users can set the environment variable
`DRAKE_DISABLE_CPU_FEATURES` to a comma-separated list of CPU features to not
use at runtime. This is the same idea as <a
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

/* Note to Drake Developers: with our current settings in common/BUILD.bazel,
for x86 there are only three flavors of Highway targets, which it names as:
- PLAIN
- AVX2
- AVX3

To force the PLAIN target at runtime, export DRAKE_DISABLE_CPU_FEATURES=AVX2.

To force the AVX2 target at runtime, export DRAKE_DISABLE_CPU_FEATURES=AVX512F
and run on a computer that actually has AVX2.

To use the AVX3 target at runtime, don't set any environment variable; just run
on a computer that actually has AVX3. */

namespace internal {

/* For use by hwy_dynamic_impl.h. Returns a target bitmask that is set to all
ones except for the bits associated with targets disallowed by the environment
variables documented above. (Therefore, in the typical case the return value
will be all ones, i.e., -1.) */
int64_t GetHighwayAllowedTargetMask();

}  // namespace internal
}  // namespace drake
