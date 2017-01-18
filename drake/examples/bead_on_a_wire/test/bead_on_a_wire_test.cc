#include "drake/examples/bead_on_a_wire/bead_on_a_wire.h"

#include <memory>

#include "gtest/gtest.h"

namespace drake {
namespace bead_on_a_wire {
namespace {

class BeadOnAWireTest : public ::testing::Test {
 protected:
  void SetUp() override {
    /// Construct the bead on the wire in absolute coordinates.
    dut_ = std::make_unique<BeadOnAWire<double>>(
        BeadOnAWire<double>::kAbsoluteCoordinates);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();
  }

  std::unique_ptr<systems::ContinuousState<double>> CreateNewContinuousState()
                                                                         const {
    const int state_dim = 6;
    auto cstate_vec = std::make_unique<systems::BasicVector<double>>(state_dim);
    return std::make_unique<systems::ContinuousState<double>>(
        std::move(cstate_vec), state_dim / 2, state_dim / 2, 0);
  }

  systems::VectorBase<double>* continuous_state() {
    return context_->get_mutable_continuous_state_vector();
  }

  std::unique_ptr<BeadOnAWire<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};

/// Checks output is as expected.
TEST_F(BeadOnAWireTest, Output) {
  const systems::ContinuousState<double>& v = *context_->get_continuous_state();
  std::unique_ptr<systems::SystemOutput<double>> output =
      dut_->AllocateOutput(*context_);
  dut_->DoCalcOutput(*context_, output.get());
  for (int i=0; i< v.size(); ++i)
    EXPECT_EQ(v[i], output->get_vector_data(0)->get_value()(i));
}

/// Tests parameter getting and setting.
TEST_F(BeadOnAWireTest, Parameters) {
  // Set parameters to non-default values.
  const double g = -1.0;
  dut_->set_gravitational_acceleration(g);
  EXPECT_EQ(dut_->get_gravitational_acceleration(), g);
}

/// Tests that sinusoidal parameter function produces the values and
/// derivatives we expect.
TEST_F(BeadOnAWireTest, Sinusoidal) {
  // Set small tolerance value.
  const double tol = std::numeric_limits<double>::epsilon()*10.0;

  // Compute the value at pi/3.
  BeadOnAWire<double>::DScalar s;
  s = M_PI/3.0;
  auto v = dut_->sinusoidal_function(s);
  const double svalue = s.value().value();
  EXPECT_NEAR(v(0).value().value(), std::cos(svalue), tol);
  EXPECT_NEAR(v(1).value().value(), std::sin(svalue), tol);
  EXPECT_NEAR(v(2).value().value(), svalue, tol);

  // Compute the derivative at pi/3.
  s.derivatives()(0) = 1.0;
  s.value().derivatives()(0) = 1.0;
  auto w = dut_->sinusoidal_function(s);
  EXPECT_EQ(w(0).value().derivatives().size(), 1);
  EXPECT_EQ(w(1).value().derivatives().size(), 1);
  EXPECT_EQ(w(2).value().derivatives().size(), 1);
  EXPECT_NEAR(w(0).value().derivatives()(0), -std::sin(svalue), tol);
  EXPECT_NEAR(w(1).value().derivatives()(0), std::cos(svalue), tol);
  EXPECT_NEAR(w(2).value().derivatives()(0), 1.0, tol);

  // Compute the second derivative at pi/3.
  const double deriv2x = w(0).derivatives()(0).derivatives()(0);
  const double deriv2y = w(1).derivatives()(0).derivatives()(0);
  const double deriv2z = w(2).derivatives()(0).derivatives()(0);
  EXPECT_NEAR(deriv2x, -std::cos(svalue), tol);
  EXPECT_NEAR(deriv2y, -std::sin(svalue), tol);
  EXPECT_NEAR(deriv2z, 0.0, tol);
}

/// Tests that the inverse of the sinusoidal parameter function produces the
/// values and derivatives we expect.
TEST_F(BeadOnAWireTest, InverseSinusoidal) {
  // Set small tolerance value.
  const double tol = std::numeric_limits<double>::epsilon()*10.0;

  // Compute the value at pi/3.
  BeadOnAWire<double>::DScalar s;
  s = M_PI/3.0;
  auto v = dut_->sinusoidal_function(s);
  v(0).derivatives().resize(1);
  v(0).derivatives()(0) = 0.0;
  v(1).derivatives().resize(1);
  v(1).derivatives()(0) = 0.0;
  v(2).derivatives().resize(1);
  v(2).derivatives()(0) = 1.0;
  auto sprime = dut_->inverse_sinusoidal_function(v);
  EXPECT_NEAR(s.value().value(), sprime.value().value(), tol);

  // Compute the derivative of the inverse sinusoidal function.
  s.derivatives()(0) = 1.0;
  s.value().derivatives()(0) = 1.0;
  auto wprime = dut_->inverse_sinusoidal_function(dut_->sinusoidal_function(s));
  const double candidate_value = wprime.value().derivatives()(0);

  // Apply numerical differentiation to the inverse sinusoidal function. We
  // do this by computing the output:
  // f⁻¹(f(s+ds)) - f⁻¹(f(s))
  // ------------------------
  //            ds
  const double ds = std::numeric_limits<double>::epsilon();
  auto fprime = dut_->inverse_sinusoidal_function(dut_->sinusoidal_function(s+ds));
  auto f = dut_->inverse_sinusoidal_function(dut_->sinusoidal_function(s));
  const double num_value = (fprime.value().value() - f.value().value())/ds;
  EXPECT_NEAR(candidate_value, num_value, tol);
}

}  // namespace
}  // namespace bead_on_a_wire
}  // namespace drake
