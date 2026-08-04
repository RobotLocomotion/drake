#include "drake/math/unit_vector.h"

#include <cmath>
#include <stdexcept>

#include "drake/common/default_scalars.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/unused.h"
#include "drake/math/autodiff.h"

namespace drake {
namespace math {
namespace internal {

template <typename T>
Vector3<T> NormalizeOrThrow(const Vector3<T>& v,
                            std::string_view function_name) {
  DRAKE_DEMAND(!function_name.empty());
  const T norm = v.norm();
  if constexpr (scalar_predicate<T>::is_bool) {
    // Throw an exception if norm is non-finite (NaN or infinity) or too small.
    // The threshold for "too small" is a heuristic (rule of thumb) guided by an
    // expected small physical dimensions in a robotic systems. Numbers smaller
    // than this are probably user or developer errors.
    constexpr double kMinMagnitude = 1e-10;
    using std::isfinite;
    if (!(isfinite(norm) && norm >= kMinMagnitude)) {
      throw std::logic_error(fmt::format(
          "{}() cannot normalize the given vector v.\n"
          "   v = {}\n"
          " |v| = {}\n"
          " The measures must be finite and the vector must have a magnitude of"
          " at least {} to normalize. If you are confident that v's direction"
          " is meaningful, pass v.normalized() instead of v.",
          function_name, fmt_eigen(DiscardGradient(v)),
          ExtractDoubleOrThrow(norm), kMinMagnitude));
    }
  }
  return v / norm;
}

template <typename T>
void ThrowIfNotUnitVector(const Vector3<T>& unit_vector,
                          std::string_view function_name,
                          const double tolerance_unit_vector_norm) {
  DRAKE_DEMAND(!function_name.empty());
  if constexpr (scalar_predicate<T>::is_bool) {
    // A test that a unit vector's magnitude is within a very small ε of 1 is
    // |√(𝐯⋅𝐯) − 1| ≤ ε. To avoid an unnecessary square-root, notice that this
    // simple test is equivalent to  ±(√(𝐯⋅𝐯) - 1) ≤ ε  which means that both
    // √(𝐯⋅𝐯) - 1 ≤ ε  and √(𝐯⋅𝐯) - 1 ≥ -ε,  or
    // √(𝐯⋅𝐯) ≤ 1 + ε  and √(𝐯⋅𝐯) ≥ 1 - ε.  Squaring these two equations give
    // 𝐯⋅𝐯 ≤ (1 + ε)²  and 𝐯⋅𝐯 ≥ (1 - ε)².  Distributing the square results in
    // 𝐯⋅𝐯 ≤ 1 + 2 ε + ε²  and 𝐯⋅𝐯 ≥ 1 - 2 ε + ε². Since ε² ≪ 2 ε, this gives
    // 𝐯⋅𝐯 - 1 ≤ 2 ε   and 𝐯⋅𝐯 - 1 ≥ -2 ε  or  |𝐯⋅𝐯 − 1| ≤ 2 ε
    // -------------------------------------------------------------
    // Hence the following test with norm() that uses an extra √, e.g., as
    // sqrt(squaredNorm()), is replaced by one that only uses squaredNorm().
    // is_ok_unit_vector =
    //     (abs(unit_vector.norm() - 1) <=  tolerance_unit_vector_norm;
    // -------------------------------------------------------------
    const double tolerance2 = 2 * tolerance_unit_vector_norm;
    const double uvec_squared = DiscardGradient(unit_vector).squaredNorm();
    const bool is_ok_unit_vector = std::isfinite(uvec_squared) &&
                                   std::abs(uvec_squared - 1.0) <= tolerance2;
    if (!is_ok_unit_vector) {
      using std::abs;
      const T norm = unit_vector.norm();
      const T norm_diff = abs(1.0 - norm);
      throw std::logic_error(fmt::format(
          "{}(): The unit_vector argument {} is not a unit vector.\n"
          "|unit_vector| = {}\n"
          "||unit_vector| - 1| = {} is greater than {}.",
          function_name, fmt_eigen(unit_vector), norm, norm_diff,
          tolerance_unit_vector_norm));
    }
  } else {
    unused(unit_vector, tolerance_unit_vector_norm);
  }
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (&NormalizeOrThrow<T>, &ThrowIfNotUnitVector<T>));

}  // namespace internal
}  // namespace math
}  // namespace drake
