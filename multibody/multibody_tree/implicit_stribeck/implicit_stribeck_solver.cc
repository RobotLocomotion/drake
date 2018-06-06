#include "drake/multibody/multibody_tree/implicit_stribeck/implicit_stribeck_solver.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/extract_double.h"

namespace drake {
namespace multibody {
namespace implicit_stribeck {

namespace internal {
template <typename T>
T LimitDirectionChange<T>::run(
    const Eigen::Ref<const Vector2<T>>& v,
    const Eigen::Ref<const Vector2<T>>& dv,
    double cos_min, double v_stribeck, double tolerance) {
  DRAKE_DEMAND(dv.size() == v.size());

  using std::abs;
  using std::sqrt;

  const double epsilon_v = v_stribeck * tolerance;
  const double epsilon_v2 = epsilon_v * epsilon_v;

  const Vector2<T> v1 = v + dv;  // v_alpha = v + dv, when alpha = 1.

  // Case I: Quick exit for small changes in v.
  const T dv_norm2 = dv.squaredNorm();
  if (dv_norm2 < epsilon_v2) {
    return 1.0;
  }

  const T v_norm = v.norm();
  const T v1_norm = v1.norm();
  const T x = v_norm / v_stribeck;    // Dimensionless slip v and v1.
  const T x1 = v1_norm / v_stribeck;
  // From Case I, we know dv_norm > epsilon_v.
  const T dv_norm = sqrt(dv_norm2);

  // Case II: limit transition from stiction to sliding when x << 1.0 and
  // gradients might be close to zero (due to the "soft norms").
  if (x < tolerance && x1 > 1.0) {
    // we know v1 != 0  since x1 > 1.0.
    // We make |v_alpha| = v_stribeck / 2.
    // For this case dv ≈ v1 (v ≈ 0). Therefore:
    // alpha ≈ v_stribeck / dv_norm / 2.
    return v_stribeck / dv_norm / 2.0;
  }

  // Another quick exit.
  if (x < 1.0) {
    // Since we went through Case II, we know that either:
    //   1. x1 < 1.0. Then we just make alpha = 1.0
    //   2. x > parameters_.tolerance i.e. we are in the zone of
    //      "strong gradients" where it is safe to take alpha = 1.0.
    return 1.0;
  } else {  // x > 1.0
    // We want to detect transition from sliding (x > 1) to stiction when the
    // velocity change passes through the circle of radius v_stribeck.

    if (x1 < 1.0) {
      // Transition happens with alpha = 1.0.
      return 1.0;
    }

    // Since v_stribeck is so small, this very seldom happens. However, it is a
    // very common case for 1D-like problems for which tangential velocities
    // change in sign and typically if we don't do anything we miss the zero
    // crossing.
    // Notice that since we passed the check in Case I, we know that:
    //  - x > 1.0
    //  - x1 > 1.0
    //  - dv_norm > epsilon_v (i.e. non-zero)
    const T v_dot_dv = v.dot(dv);
    if (v_dot_dv < 0.0) {  // Moving towards the origin.
      T alpha = -v_dot_dv / dv_norm2;  // alpha > 0
      if (alpha < 1.0) {  // we might have missed a cross by zero. Check.
        const Vector2<T> v_alpha = v + alpha * dv;  // v1.dot(v) = 0.
        const T v_alpha_norm = v_alpha.norm();

        // Within the circle of radius v_stribeck, but almost zero.
        if (v_alpha_norm < epsilon_v) {
          return alpha - v_stribeck / 2.0 / dv_norm;
        } else if (v_alpha_norm < v_stribeck) {
          return alpha;
        }
      }
    }

    // If we are here we know:
    //  - x > 1.0
    //  - x1 > 1.0
    //  - dv_norm > epsilon_v
    //  - line connecting v with v1 never goes through circle of radius
    //    v_stribeck.
    //
    // Case III:
    // Therefore we know changes happen entirely outside the circle of radius
    // v_stribeck. To avoid large jumps in the direction of v (typically during
    // strong impacts), we limit the maximum angle change between v to v1.
    // To this end, we find a scalar 0 < alpha < 1 such that
    // cos(theta_max) = v⋅v1/(‖v‖‖v1‖).

    // First we compute the angle change when alpha = 1, between v1 and v.
    const T cos1 = v.dot(v1) / v_norm / v1_norm;

    // We allow angle changes theta < theta_max, and we take alpha = 1.0.
    // In particular, when v1 is exactly aligned wiht v (but we know it does not
    // cross through zero, i.e. cos(theta) > 0).
    if (cos1 > cos_min) {
      return 1.0;
    } else {  // we limit the angle change to theta_max.
      // All term are made non-dimensional using v_stribeck as the reference
      // sale.
      const T x_dot_dx = v_dot_dv / (v_stribeck * v_stribeck);
      const T dx = dv.norm() / v_stribeck;
      const T dx2 = x * x;
      const T dx4 = dx2 * dx2;
      const T cmin2 = cos_min * cos_min;

      // Form the terms of the quadratic equation aα² + bα + c = 0.
      const T a = dx2 * dx * dx * cmin2 - x_dot_dx * x_dot_dx;
      const T b = 2 * dx2 * x_dot_dx * (cmin2 - 1.0);
      const T c = dx4 * (cmin2 - 1.0);

      // Solve quadratic equation. We know, from the geometry of the problem,
      // that the roots to this problem are real. Thus, the discriminant of the
      // quadratic equation (Δ = b² - 4ac) is positive.
      // Also, unless there is a single root (a = 0), they have different signs.
      // From Vieta's formulas we know:
      //   - α₁ + α₂ = -b / a
      //   - α₁ * α₂ = c / a < 0
      // Since we know that c < 0 (strictly), then we must have a ≥ 0.
      // Also since c < 0 strictly, α = 0 (zero) cannot be a root.
      // We use this knowledge in the solution below.

      T alpha;
      // First determine if a = 0 (to machine epsilon).
      if (abs(a) < std::numeric_limits<double>::epsilon()) {
        // There is only a single root to the, now linear, equation bα + c = 0.
        alpha = -c / b;
        // Note: a = 0, α > 0 => x_dot_dx = x * dx * cmin ≠ 0 => b ≠ 0
      } else {
        // The determinant of the quadratic equation.
        const T Delta = b * b - 4 * a * c;
        // Geometry tell us that a real solution does exist i.e. Delta > 0.
        DRAKE_ASSERT(Delta > 0);
        const T sqrt_delta = sqrt(Delta);
        alpha = (-b + sqrt_delta) / a / 2.0;  // we know a > 0
      }

      // The geometry of the problem tells us that α ≤ 1.0
      DRAKE_ASSERT(alpha <= 1.0);
      return alpha;
    }
  }

  // We should never reach this point.
  throw std::logic_error("Bug detected. An angle change case was missed.");
}
}

}  // namespace implicit_stribeck
}  // namespace multibody
}  // namespace drake

// Explicitly instantiates on the most common scalar types.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    struct
    ::drake::multibody::implicit_stribeck::internal::LimitDirectionChange)
