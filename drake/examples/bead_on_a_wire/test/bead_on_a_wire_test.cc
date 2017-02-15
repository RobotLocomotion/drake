#include "drake/examples/bead_on_a_wire/bead_on_a_wire.h"

#include <memory>

#include "gtest/gtest.h"

namespace drake {
namespace examples {
namespace bead_on_a_wire {
namespace {

class BeadOnAWireTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Construct beads on the wire in absolute and minimal coordinates.
    dut_abs_ = std::make_unique<BeadOnAWire<double>>(
        BeadOnAWire<double>::kAbsoluteCoordinates);
    dut_min_ = std::make_unique<BeadOnAWire<double>>(
        BeadOnAWire<double>::kMinimalCoordinates);

    // Construct contexts.
    context_abs_ = dut_abs_->CreateDefaultContext();
    context_min_ = dut_min_->CreateDefaultContext();

    // Construct outputs.
    output_abs_ = dut_abs_->AllocateOutput(*context_abs_);
    output_min_ = dut_min_->AllocateOutput(*context_abs_);

    // Allocate storage for derivatives.
    derivatives_abs_ = dut_abs_->AllocateTimeDerivatives();
    derivatives_min_ = dut_min_->AllocateTimeDerivatives();

    // Fix unused input ports.
    std::unique_ptr<systems::BasicVector<double>> input_abs =
        std::make_unique<systems::BasicVector<double>>(3);
    std::unique_ptr<systems::BasicVector<double>> input_min =
        std::make_unique<systems::BasicVector<double>>(1);
    input_abs->SetAtIndex(0, 0.0);
    input_abs->SetAtIndex(1, 0.0);
    input_abs->SetAtIndex(2, 0.0);
    input_min->SetAtIndex(0, 0.0);
    context_abs_->FixInputPort(0, std::move(input_abs));
    context_min_->FixInputPort(0, std::move(input_min));
  }

  std::unique_ptr<BeadOnAWire<double>> dut_abs_;  //< A device under test.
  std::unique_ptr<BeadOnAWire<double>> dut_min_;  //< A device under test.
  std::unique_ptr<systems::Context<double>> context_abs_, context_min_;
  std::unique_ptr<systems::SystemOutput<double>> output_abs_, output_min_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_abs_,
                                                    derivatives_min_;
};

// Checks output is as expected.
TEST_F(BeadOnAWireTest, Output) {
  const systems::ContinuousState<double>& v = *context_abs_->
                                               get_continuous_state();
  dut_abs_->CalcOutput(*context_abs_, output_abs_.get());
  for (int i = 0; i < v.size(); ++i)
    EXPECT_EQ(v[i], output_abs_->get_vector_data(0)->get_value()(i));
}

// Tests parameter getting and setting.
TEST_F(BeadOnAWireTest, Parameters) {
  // Set parameters to non-default values.
  const double g = -1.0;
  dut_abs_->set_gravitational_acceleration(g);
  EXPECT_EQ(dut_abs_->get_gravitational_acceleration(), g);
}

// Tests that helix parameter function produces the values and
// derivatives we expect.
TEST_F(BeadOnAWireTest, Helix) {
  // Set small tolerance value.
  const double tol = std::numeric_limits<double>::epsilon() * 10.0;

  // Compute the value at pi/3.
  BeadOnAWire<double>::DScalar s;
  s = M_PI / 3.0;
  auto v = dut_abs_->helix_function(s);
  const double svalue = s.value().value();
  EXPECT_NEAR(v(0).value().value(), std::cos(svalue), tol);
  EXPECT_NEAR(v(1).value().value(), std::sin(svalue), tol);
  EXPECT_NEAR(v(2).value().value(), svalue, tol);

  // Compute the derivative at pi/3.
  s.derivatives()(0) = 1.0;
  s.value().derivatives()(0) = 1.0;
  auto w = dut_abs_->helix_function(s);
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

// Tests that the inverse of the helix parameter function produces the
// values and derivatives we expect.
TEST_F(BeadOnAWireTest, InverseHelix) {
  // Set small tolerance value.
  const double tol = std::numeric_limits<double>::epsilon() * 10.0;

  // Compute the value at pi/3.
  const double test_value = M_PI / 3.0;
  BeadOnAWire<double>::DScalar s;
  s = test_value;
  auto v = dut_abs_->helix_function(s);
  v(0).derivatives()(0) = 0.0;
  v(1).derivatives()(0) = 0.0;
  v(2).derivatives()(0) = 0.0;
  auto sprime = dut_abs_->inverse_helix_function(v);
  EXPECT_NEAR(s.value().value(), sprime.value().value(), tol);

  // Compute the derivative of the inverse helix function.
  s.derivatives()(0) = 1.0;
  s.value().derivatives()(0) = 1.0;
  auto wprime = dut_abs_->inverse_helix_function(dut_abs_->
                                                      helix_function(s));
  const double candidate_value = wprime.value().derivatives()(0);

  // Apply numerical differentiation to the inverse helix function. We
  // do this by computing the output:
  // f⁻¹(f(s+ds)) - f⁻¹(f(s))
  // ------------------------
  //            ds
  const double ds = std::numeric_limits<double>::epsilon();
  auto fprime = dut_abs_->inverse_helix_function(
      dut_abs_->helix_function(s + ds));
  auto f = dut_abs_->inverse_helix_function(dut_abs_->
      helix_function(s));
  const double num_value = (fprime.value().value() - f.value().value()) / ds;
  EXPECT_NEAR(candidate_value, num_value, tol);

  // Compute the derivative of the inverse helix function using the
  // velocity.
  const double x = std::cos(test_value);
  const double y = std::sin(test_value);
  const double z = test_value;
  const double xdot = 1.0;
  const double ydot = 2.0;
  const double zdot = 3.0;
  const double inv_helix_dot = -y / (x * x + y * y) * xdot +
      x / (x * x + y * y) * ydot;
  Eigen::Matrix<BeadOnAWire<double>::DScalar, 3, 1> xx;
  xx(0).value() = x;
  xx(1).value() = y;
  xx(2).value() = z;
  xx(0).value().derivatives()(0) = xdot;
  xx(1).value().derivatives()(0) = ydot;
  xx(2).value().derivatives()(0) = zdot;
  auto tprime = dut_abs_->inverse_helix_function(xx);
  EXPECT_NEAR(inv_helix_dot, tprime.value().derivatives()(0), tol);
}

// Tests the constraint function evaluation using the helix function.
TEST_F(BeadOnAWireTest, ConstraintFunctionEval) {
  // Put the bead directly onto the wire.
  systems::ContinuousState<double> &xc =
      *context_abs_->get_mutable_continuous_state();
  const double s = 1.0;
  xc[0] = std::cos(s);
  xc[1] = std::sin(s);
  xc[2] = s;

  // Verify that the constraint error is effectively zero.
  const double tol = std::numeric_limits<double>::epsilon() * 10;
  EXPECT_NEAR(dut_abs_->EvalConstraintEquations(*context_abs_).norm(),
              0.0, tol);

  // Move the bead off of the wire and verify limit on expected error.
  const double err = 1.0;
  xc[0] += err;
  EXPECT_LE(dut_abs_->EvalConstraintEquations(*context_abs_).norm(), 1.0);
  xc[0] = std::cos(s);
  xc[1] += err;
  EXPECT_LE(dut_abs_->EvalConstraintEquations(*context_abs_).norm(), 1.0);
}

// Tests the evaluation of the time derivative of the constraint functions
// using the helix function.
TEST_F(BeadOnAWireTest, ConstraintDotFunctionEval) {
  // Put the bead directly onto the wire and make its velocity such that
  // it is not instantaneously leaving the wire.
  systems::ContinuousState<double> &xc =
      *context_abs_->get_mutable_continuous_state();
  const double s = 1.0;
  xc[0] = std::cos(s);
  xc[1] = std::sin(s);
  xc[2] = s;
  xc[3] = -std::sin(s);
  xc[4] = std::cos(s);
  xc[5] = 1.0;

  // Verify that the constraint error is effectively zero.
  const double tol = std::numeric_limits<double>::epsilon() * 10;
  EXPECT_NEAR(dut_abs_->EvalConstraintEquationsDot(*context_abs_).norm(),
              0.0, tol);
}

static Eigen::Matrix<BeadOnAWire<double>::DScalar, 3, 1>
  horz_line_function(const BeadOnAWire<double>::DScalar& s) {
  return Vector3<BeadOnAWire<double>::DScalar>(s, s, s*0);
}

static BeadOnAWire<double>::DScalar inverse_horz_line_function(
        const Vector3<BeadOnAWire<double>::DScalar>& v) {
  return (v(1)+v(0))/2;
}

static Eigen::Matrix<BeadOnAWire<double>::DScalar, 3, 1>
  vert_line_function(const BeadOnAWire<double>::DScalar& s) {
  return Vector3<BeadOnAWire<double>::DScalar>(s*0, s*0, s);
}

static BeadOnAWire<double>::DScalar inverse_vert_line_function(
        const Vector3<BeadOnAWire<double>::DScalar>& v) {
  return v(2);
}

// Verify that the minimal coordinate dynamics computations are reasonable.
TEST_F(BeadOnAWireTest, MinimalCoordinateDeriv) {
  const int n_vars = derivatives_min_->get_vector().size();
  const double tol = std::numeric_limits<double>::epsilon();

  // For horizontal wire function, velocity derivatives should be zero.
  dut_min_->reset_wire_parameter_functions(&horz_line_function,
                                           &inverse_horz_line_function);
  dut_min_->CalcTimeDerivatives(*context_min_, derivatives_min_.get());
  Eigen::VectorXd deriv = derivatives_min_->get_vector().CopyToVector();
  EXPECT_LT(deriv.segment(n_vars/2, n_vars/2).norm(), tol);

  // For vertical wire function, velocity derivatives should be gravitational
  // acceleration.
  dut_min_->reset_wire_parameter_functions(&vert_line_function,
                                           &inverse_vert_line_function);
  dut_min_->CalcTimeDerivatives(*context_min_, derivatives_min_.get());
  deriv = derivatives_min_->get_vector().CopyToVector();
  EXPECT_NEAR(deriv.segment(n_vars/2, n_vars/2).norm(),
              std::abs(dut_min_->get_gravitational_acceleration()), tol);
}

}  // namespace
}  // namespace bead_on_a_wire
}  // namespace examples
}  // namespace drake
