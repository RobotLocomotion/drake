#include "drake/multibody/benchmarks/mass_damper_spring/mass_damper_spring_analytical_solution.h"

#include <limits>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace multibody {
namespace benchmarks {

// For `this` mass-damper-spring system, and with the given initial
// values, this method calculates the values of x, ẋ, ẍ at time t.
template <typename T>
Vector3<T> MassDamperSpringAnalyticalSolution<T>::CalculateOutput(
    const T& t) const {
  // TODO(@mitiguy) Enhance algorithm to allow for any real values of m, b, k,
  // (except m = 0), e.g., to allow for unstable control systems.
  DRAKE_DEMAND(m_ > 0  &&  b_ >= 0  &&  k_ > 0);

  const T zeta = CalculateDampingRatio();
  const T wn = CalculateNaturalFrequency();
  return CalculateOutputImpl(zeta, wn, x0_, xDt0_, t);
}

// Calculates the values of x, ẋ, ẍ at time t associated with the ODE
// ẍ  +  2 ζ ωₙ ẋ  +  ωₙ²  =  0  and the given initial values.
// Algorithm from [Kane, 1985] Problem Set 14.7-14.10, Pages 349-352.
//
// - [Kane, 1985] "Dynamics: Theory and Applications," McGraw-Hill Book Co.,
//   New York, 1985 (with D. A. Levinson).  Available for free .pdf download:
//   https://ecommons.cornell.edu/handle/1813/637
template <typename T>
Vector3<T> MassDamperSpringAnalyticalSolution<T>::CalculateOutputImpl(
    const T& zeta, const T& wn, const T& x0, const T& xDt0,  const T& t) {
  // TODO(@mitiguy) Enhance algorithm to allow for any real values of zeta, wn.
  DRAKE_DEMAND(zeta >= 0  &&  wn > 0);

  // Quantities x, ẋ, ẍ are put into a three-element matrix and returned.
  T x, xDt, xDtDt;

  using std::abs;
  using std::exp;
  using std::sqrt;
  using std::cos;
  using std::sin;

  constexpr double epsilon = std::numeric_limits<double>::epsilon();
  const bool is_zeta_nearly_1 = abs(zeta - 1) < 10 * epsilon;
  if (is_zeta_nearly_1) {
    // Critically damped free vibration (zeta = 1).
    const T A = x0;
    const T B = xDt0 + wn * x0;

    const T factor1 = A + B * t;
    const T factor2 = exp(-wn * t);
    x = factor1 * factor2;

    const T factor1Dt = B;
    const T factor2Dt = -wn * exp(-wn * t);
    xDt = factor1Dt * factor2 + factor1 * factor2Dt;

    const T factor2DtDt = wn * wn * exp(-wn * t);
    xDtDt = 2 * factor1Dt * factor2Dt + factor1 * factor2DtDt;
  } else if (zeta < 1) {
    // Undamped or underdamped free vibration (0 <= zeta < 1).
    const T wd = wn * sqrt(1 - zeta * zeta);  // Damped natural frequency.
    const T A = (xDt0 + zeta * wn * x0) / wd;
    const T B = x0;

    const T factor1 = A * sin(wd * t) + B * cos(wd * t);
    const T factor2 = exp(-zeta * wn * t);
    x = factor1 * factor2;

    const T factor1Dt = A * wd * cos(wd * t) - B * wd * sin(wd * t);
    const T factor2Dt = -zeta * wn * exp(-zeta * wn * t);
    xDt = factor1Dt * factor2 + factor1 * factor2Dt;

    const T factor1DtDt = -wd * wd * factor1;
    const T factor2DtDt = (-zeta*wn) * (-zeta*wn) * exp(-zeta * wn * t);
    xDtDt = factor1DtDt*factor2 + 2*factor1Dt*factor2Dt + factor1*factor2DtDt;
  } else {
    DRAKE_DEMAND(zeta > 1);
    // Overdamped  free vibration (zeta > 1).
    const T p1 = -wn * (zeta - sqrt(zeta * zeta - 1));
    const T p2 = -wn * (zeta + sqrt(zeta * zeta - 1));
    const T A = (xDt0 - p2 * x0) / (p1 - p2);
    const T B = (xDt0 - p1 * x0) / (p1 - p2);

    const T term1 = A * exp(p1 * t);
    const T term2 = B * exp(p2 * t);
    x = term1 - term2;

    const T term1Dt = A * p1 * exp(p1 * t);
    const T term2Dt = B * p2 * exp(p2 * t);
    xDt = term1Dt - term2Dt;

    const T term1DtDt = A * p1 * p1 * exp(p1 * t);
    const T term2DtDt = B * p2 * p2 * exp(p2 * t);
    xDtDt = term1DtDt - term2DtDt;
  }

  Vector3<T> output;
  output << x, xDt, xDtDt;
  return output;
}

}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::benchmarks::MassDamperSpringAnalyticalSolution)
