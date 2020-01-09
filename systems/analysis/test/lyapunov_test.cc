#include "drake/systems/analysis/lyapunov.h"

#include <cmath>

#include <gtest/gtest.h>

#include "drake/examples/pendulum/pendulum_plant.h"
#include "drake/systems/framework/vector_system.h"

namespace drake {
namespace systems {
namespace analysis {
namespace {

using symbolic::Variable;
using symbolic::Expression;

/// Cubic Polynomial System:
///   ẋ = -x + x³
///   y = x
class CubicPolynomialSystem : public systems::VectorSystem<double> {
 public:
  CubicPolynomialSystem()
      : systems::VectorSystem<double>(0, 0) {  // Zero inputs, zero outputs.
    this->DeclareContinuousState(1);           // One state variable.
  }

 private:
  // ẋ = -x + x³
  virtual void DoCalcVectorTimeDerivatives(
      const systems::Context<double>&,
      const Eigen::VectorBlock<const Eigen::VectorXd>&,
      const Eigen::VectorBlock<const Eigen::VectorXd>& state,
      Eigen::VectorBlock<Eigen::VectorXd>* derivatives) const {
    using std::pow;
    (*derivatives)(0) = -state(0) + pow(state(0), 3.0);
  }
};

VectorX<AutoDiffXd> cubic_bases(VectorX<AutoDiffXd> x) {
  return Vector1<AutoDiffXd>{x.dot(x)};
}

// Verifies by only taking samples inside the region of attraction of the
// origin, x ∈ [-1, 1].  This is taken from the example in
// http://underactuated.mit.edu/underactuated.html?chapter=lyapunov .
GTEST_TEST(LyapunovTest, CubicPolynomialTest) {
  CubicPolynomialSystem cubic;
  auto context = cubic.CreateDefaultContext();
  Eigen::RowVectorXd x_samples = Eigen::RowVectorXd::LinSpaced(21, -1., 1.);

  Eigen::VectorXd params = SampleBasedLyapunovAnalysis(
      cubic, *context, &cubic_bases, x_samples, Vector1d{0.});

  // Make sure that the solver found some reasonable, well-conditioned answer.
  EXPECT_GE(params(0), .1);
  EXPECT_LE(params(0), 10.);
}

template <typename T>
VectorX<T> pendulum_bases(const VectorX<T>& x) {
  const T s = sin(x[0]);
  const T c = cos(x[0]);
  const T qd = x[1];

  VectorX<T> monomials(9);
  monomials << 1, s, c, qd, s * s, s * c, s * qd, c * qd, qd * qd;
  return monomials;
}

GTEST_TEST(LyapunovTest, PendulumSampleBasedLyapunov) {
  examples::pendulum::PendulumPlant<double> pendulum;
  auto context = pendulum.CreateDefaultContext();
  pendulum.get_input_port().FixValue(context.get(), 0.0);

  Eigen::VectorXd q_samples =
      Eigen::VectorXd::LinSpaced(31, -1.5 * M_PI, 1.5 * M_PI);
  Eigen::VectorXd qd_samples = Eigen::VectorXd::LinSpaced(21, -10., 10.);
  Eigen::Matrix2Xd x_samples(2, q_samples.size() * qd_samples.size());
  int xi = 0;
  for (int qi = 0; qi < q_samples.size(); qi++) {
    for (int qdi = 0; qdi < qd_samples.size(); qdi++) {
      x_samples.col(xi++) = Eigen::Vector2d{q_samples(qi), qd_samples(qdi)};
    }
  }
  // V(0) = 0.
  const Eigen::Vector2d x_zero = Eigen::Vector2d::Zero();

  Eigen::VectorXd params = SampleBasedLyapunovAnalysis(
      pendulum, *context, &pendulum_bases<AutoDiffXd>, x_samples, x_zero);

  // Zero the small coefficients, for textual display.
  for (int i = 0; i < params.size(); i++) {
    if (fabs(params(i)) < 1e-4) {
      params(i) = 0.;
    }
  }
  // Check if V ends up near a constant factor times mechanical energy:
  const auto& p = pendulum.get_parameters(*context);
  Eigen::VectorXd energy_params = Eigen::VectorXd::Zero(9);
  energy_params(0) = p.mass() * p.gravity() * p.length();
  energy_params(2) = -p.mass() * p.gravity() * p.length();
  energy_params(8) = p.mass() * p.length() * p.length();

  const double factor = energy_params(0) / params(0);
  params *= factor;
  EXPECT_NEAR(params[2], energy_params[2], 1e-4);

  Vector2<Expression> state{Variable("q"), Variable("qd")};
  drake::log()->info(
      "E = " +
      energy_params.dot(pendulum_bases<Expression>(state)).to_string());
  drake::log()->info("V = " +
                     params.dot(pendulum_bases<Expression>(state)).to_string());
}

}  // namespace
}  // namespace analysis
}  // namespace systems
}  // namespace drake
