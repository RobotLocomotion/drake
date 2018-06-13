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
T DirectionChangeLimiter<T>::CalcAlpha(
    const Eigen::Ref<const Vector2<T>>& v,
    const Eigen::Ref<const Vector2<T>>& dv,
    double cos_theta_max, double v_stiction, double relative_tolerance) {
  DRAKE_ASSERT(v_stiction > 0);
  DRAKE_ASSERT(relative_tolerance > 0);
  DRAKE_ASSERT(dv.size() == v.size());

  using std::abs;
  using std::max;
  using std::min;
  using std::sqrt;

  // εᵥ is used to determine when a velocity is close to zero.
  const double epsilon_v = v_stiction * relative_tolerance;
  const double epsilon_v2 = epsilon_v * epsilon_v;

  const Vector2<T> v1 = v + dv;  // v_alpha = v + dv, when alpha = 1.

  // Case I: Quick exit for small changes in v.
  const T dv_norm2 = dv.squaredNorm();
  if (dv_norm2 < epsilon_v2) {
    return 1.0;
  }

  const T v_norm = v.norm();
  const T v1_norm = v1.norm();
  const T x = v_norm / v_stiction;    // Dimensionless slip v and v1.
  const T x1 = v1_norm / v_stiction;
  // From Case I, we know dv_norm > epsilon_v.
  const T dv_norm = sqrt(dv_norm2);

  // Case II: limit transition from stiction to sliding when x << 1.0 and
  // gradients might be close to zero (due to the "soft norms").
  if (x < relative_tolerance && x1 > 1.0) {
    // we know v1 != 0  since x1 > 1.0.
    // With v_alpha = v + alpha * dv, we make |v_alpha| = v_stiction / 2.
    // For this case dv ≈ v1 (v ≈ 0). Therefore:
    // alpha ≈ v_stiction / dv_norm / 2.
    return v_stiction / dv_norm / 2.0;
  }

  // Case III: Transition to an almost exact stiction from sliding.
  // We want to avoid v1 landing in a region of zero gradients so we force
  // it to land within the circle of radius v_stiction, at v_stribeck/2 in the
  // direction of v.
  if (x > 1.0 && x1 < relative_tolerance) {
    // In this case x1 is negligible compared to x. That is dv ≈ -v. For this
    // case we'll limit v + αdv = vₛ/2⋅v/‖v‖. Using v ≈ -dv, we arrive to
    // dv(α-1) = -vₛ/2⋅dv/‖dv‖ or:
    return 1.0 - v_stiction / 2.0 / dv_norm;
  }

  if (x < 1.0) {
    // Another quick exit. Two possibilities (both of which yield the same
    // action):
    // x1 < 1: we go from within the Stribeck circle back into it. Since this
    //         region has strong gradients, we allow it. i.e. alpha = 1.0
    // x1 > 1: If we go from a region of strong gradients (x < 1) to sliding
    //         (x1 > 1), we allow it. Notice that the case from weak gradients
    //         (close to zero) when x < relative_tolerance, was covered by
    //         Case II.
    return 1.0;
  } else {  // x > 1.0
    if (x1 < 1.0) {
      // Case IV:
      // From Case III we know that x1 > relative_tolerance, i.e x1 falls in a
      // region of strong gradients and thus we allow it.
      return 1.0;
    }

    // Case V:
    // Since v_stiction is so small, the next case very seldom happens. However,
    // it is a very common case for 1D-like problems for which tangential
    // velocities change in sign and the zero crossing might be missed.
    // Notice that since we reached this point, we know that:
    //  - x > 1.0 (we are within the scope of an if statement for x > 1)
    //  - x1 > 1.0 (we went through Case IV)
    //  - dv_norm > epsilon_v (we went through Case I, i.e. non-zero)
    // Here we are checking for the case when the line connecting v and v1
    // intersects the Stribeck circle. For this case we compute alpha so that
    // the update corresponds to the velocity closest to the origin.
    T alpha;
    const T v_dot_dv = v.dot(dv);
    if (CrossesTheStictionRegion(
        v, dv, v_dot_dv, dv_norm, dv_norm2, epsilon_v, v_stiction, &alpha)) {
      return alpha;
    }

    // If we are here we know:
    //  - x > 1.0
    //  - x1 > 1.0
    //  - dv_norm > epsilon_v
    //  - line connecting v with v1 never goes through the Stribeck circle.
    //
    // Case VI:
    // Therefore we know changes happen entirely outside the circle of radius
    // v_stiction. To avoid large jumps in the direction of v (typically during
    // strong impacts), we limit the maximum angle change between v to v1.
    // To this end, we find a scalar 0 < alpha < 1 such that
    // cos(θₘₐₓ) = v⋅(v+αdv)/(‖v‖‖v+αdv‖), see [Uchida et al., 2015].

    // First we compute the angle change when alpha = 1, between v1 and v.
    const T cos1 = v.dot(v1) / v_norm / v1_norm;

    // We allow angle changes theta < theta_max, and we take alpha = 1.0.
    // In particular, when v1 is exactly aligned with v (but we know it does not
    // cross through zero, i.e. cos(theta) > 0).
    if (cos1 > cos_theta_max) {
      return 1.0;
    } else {
      // we limit the angle change to theta_max so that:
      // cos(θₘₐₓ) = v⋅(v+αdv)/(‖v‖‖v+αdv‖)
      // if we square both sides of this equation, we arrive at a quadratic
      // equation with coefficients a, b, c, for α. The math below simply is the
      // algebra to compute coefficients a, b, c and solve the quadratic
      // equation.

      // All terms are made non-dimensional using v_stiction as the reference
      // scale.
      const T x_dot_dx = v_dot_dv / (v_stiction * v_stiction);
      const T dx = dv.norm() / v_stiction;
      const T x2 = x * x;
      const T dx4 = x2 * x2;
      const T cos_theta_max2 = cos_theta_max * cos_theta_max;

      // Form the terms of the quadratic equation aα² + bα + c = 0.
      const T a = x2 * dx * dx * cos_theta_max2 - x_dot_dx * x_dot_dx;
      const T b = 2 * x2 * x_dot_dx * (cos_theta_max2 - 1.0);
      const T c = dx4 * (cos_theta_max2 - 1.0);

      // Solve quadratic equation. We know, from the geometry of the problem,
      // that the roots to this problem are real. Thus, the discriminant of the
      // quadratic equation (Δ = b² - 4ac) must be positive.
      // We use a very specialized quadratic solver for this case where we know
      // there must exist a positive (i.e. real) root.
      alpha = SolveQuadraticForTheSmallestPositiveRoot(a, b, c);

      // The geometry of the problem tells us that α ≤ 1.0
      DRAKE_ASSERT(alpha <= 1.0);
      return alpha;
    }
  }

  // We should never reach this point.
  throw std::logic_error("Bug detected. An angle change case was missed.");
}

template <typename T>
bool DirectionChangeLimiter<T>::CrossesTheStictionRegion(
    const Eigen::Ref<const Vector2<T>>& v,
    const Eigen::Ref<const Vector2<T>>& dv,
    const T& v_dot_dv,  const T& dv_norm, const T& dv_norm2,
    double epsilon_v, double v_stiction, T* alpha_out) {
  DRAKE_ASSERT(dv_norm > 0);
  DRAKE_ASSERT(dv_norm2 > 0);
  DRAKE_ASSERT(epsilon_v > 0);
  DRAKE_ASSERT(v_stiction > 0);
  T& alpha = *alpha_out;
  if (v_dot_dv < 0.0) {  // Moving towards the origin.
    alpha = -v_dot_dv / dv_norm2;  // alpha > 0
    if (alpha < 1.0) {  // The update might be crossing the stiction region.
      const Vector2<T> v_alpha = v + alpha * dv;  // Note: v_alpha.dot(dv) = 0.
      const T v_alpha_norm = v_alpha.norm();
      if (v_alpha_norm < epsilon_v) {
        // v_alpha is almost zero.
        // This situation happens when dv ≈ -a v with a > 0. Therefore we cap
        // v_alpha to v_alpha = v / ‖v‖⋅vₛ/2. To do this, we "move" v_alpha
        // in the direction opposite of dv/‖dv‖ by magnitude vs/2, or similarly,
        // we subtract ‖dv‖vs/2 from the previously computed alpha.
        alpha -= v_stiction / 2.0 / dv_norm;
        DRAKE_ASSERT(0 < alpha && alpha <= 1);
        return true;  // Crosses the stiction region.
      } else if (v_alpha_norm < v_stiction) {
        // v_alpha falls within the Stribeck circle but its magnitude is
        // larger than epsilon_v.
        return true;  // Crosses the stiction region.
      }
    }
  }
  return false;
}

template <typename T>
T DirectionChangeLimiter<T>::SolveQuadraticForTheSmallestPositiveRoot(
    const T& a, const T& b, const T& c) {
  using std::abs;
  using std::max;
  using std::min;
  using std::sqrt;
  T alpha;
  // First determine if a = 0 (to machine epsilon). This comparison is fair
  // since a is dimensionless.
  if (abs(a) < std::numeric_limits<double>::epsilon()) {
    // There is only a single root to the, now linear, equation bα + c = 0.
    alpha = -c / b;
    // Note: a = 0, α > 0 => x_dot_dx = x * dx * cmin ≠ 0 => b ≠ 0
    // We assert this even though within the scope this method is called it is
    // not possible for b to be zero.
    DRAKE_ASSERT(abs(b) > std::numeric_limits<double>::epsilon());
  } else {
    // The determinant, Δ = b² - 4ac, of the quadratic equation.
    const T Delta = b * b - 4 * a * c;  // Uppercase, as in Δ.
    // Geometry tell us that a real solution does exist i.e. Delta > 0.
    DRAKE_DEMAND(Delta > 0);
    const T sqrt_Delta = sqrt(Delta);

    // To avoid loss of significance, when 4ac is relatively small compared
    // to b² (i.e. the square root of the discriminant is close to b), we use
    // Vieta's formula (α₁α₂ = c / a) to compute the second root given we
    // computed the first root without precision lost. This guarantees the
    // numerical stability of the method.
    const T numerator = -0.5 * (b + (b > 0.0 ? sqrt_Delta : -sqrt_Delta));
    const T alpha1 = numerator / a;
    const T alpha2 = c / numerator;

    // The geometry of the problem tells us that at least one must be
    // positive.
    DRAKE_DEMAND(alpha2 > 0 || alpha1 > 0);

    if (alpha2 > 0 && alpha1 > 0) {
      // This branch is triggered for large angle changes (typically close
      // to 180 degrees) between v1 and vt.
      alpha = min(alpha2, alpha1);
    } else {
      // This branch is triggered for small angles changes (typically
      // smaller than 90 degrees) between v1 and vt.
      alpha = max(alpha2, alpha1);
    }
  }
  return alpha;
}

}  // namespace internal
}  // namespace implicit_stribeck
}  // namespace multibody
}  // namespace drake

// Explicitly instantiates on the most common scalar types.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    struct
    ::drake::multibody::implicit_stribeck::internal::DirectionChangeLimiter)
