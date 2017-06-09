#include "drake/multibody/benchmarks/mass_damper_spring/mass_damper_spring_analytical_solution.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace multibody {
namespace benchmarks {

Eigen::Vector3d MassDamperSpringAnalyticalSolution::CalculateOutput(
    const double t) const {
  DRAKE_DEMAND(m_ > 0  &&  b_ >= 0  &&  k_ > 0);

  // Returned values x, x', x''.
  double x, xDt, xDtDt;

  using std::sqrt;
  using std::sin;
  using std::cos;
  using std::exp;
  const double wn = sqrt(k_ / m_);                // Natural frequency.
  const double zeta = b_ / (2 * sqrt(m_ * k_));   // Damping ratio.

  if (zeta <= 1) {
    // Undamped or underdamped (0 <= zeta < 1) free vibration.
    const double wd = wn * sqrt(1 - zeta * zeta);  // Damped natural frequency.
    const double A = (xDt0_ + zeta * wn * x0_) / wd;
    const double B = x0_;

    const double factor1 = A * sin(wd * t) + B * cos(wd * t);
    const double factor2 = exp(-zeta * wn * t);
    x = factor1 * factor2;

    const double factor1Dt = A * wd * cos(wd * t) - B * wd * sin(wd * t);
    const double factor2Dt = -zeta * wn * exp(-zeta * wn * t);
    xDt = factor1Dt * factor2 + factor1 * factor2Dt;

    const double factor1DtDt = -wd * wd * factor1;
    const double factor2DtDt = (-zeta*wn) * (-zeta*wn) * exp(-zeta * wn * t);
    xDtDt = factor1DtDt*factor2 + 2*factor1Dt*factor2Dt + factor1*factor2DtDt;
  } else if (zeta == 1) {
    // Critically damped (zeta = 1) free vibration.
    const double A = x0_;
    const double B = xDt0_ + wn * x0_;

    const double factor1 = A + B*t;
    const double factor2 = exp(-wn * t);
    x = factor1 * factor2;

    const double factor1Dt = B;
    const double factor2Dt = -wn * exp(-wn * t);
    xDt = factor1Dt * factor2 + factor1 * factor2Dt;

    const double factor2DtDt = wn * wn * exp(-wn * t);
    xDtDt = 2 * factor1Dt * factor2Dt + factor1 * factor2DtDt;
  } else if (zeta > 1) {
    // Overdamped (zeta > 1) free vibration.
    const double p1 = -wn * (zeta - sqrt(zeta * zeta - 1));
    const double p2 = -wn * (zeta + sqrt(zeta * zeta - 1));
    const double A = xDt0_ - p2 * x0_ / (p1 - p2);
    const double B = xDt0_ - p1 * x0_ / (p1 - p2);

    const double term1 = A * exp(p1 * t);
    const double term2 = B * exp(p2 * t);
    x = term1 - term2;

    const double term1Dt = A * p1 * exp(p1 * t);
    const double term2Dt = B * p2 * exp(p2 * t);
    xDt = term1Dt - term2Dt;

    const double term1DtDt = A * p1 * p1 * exp(p1 * t);
    const double term2DtDt = B * p2 * p2 * exp(p2 * t);
    xDtDt = term1DtDt - term2DtDt;
  }

  Eigen::Vector3d output;
  output << x, xDt, xDtDt;
  return output;
}

}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake

