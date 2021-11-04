#include "drake/examples/bead_on_a_wire/bead_on_a_wire.h"

#include <memory>

#include <gtest/gtest.h>

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
    output_abs_ = dut_abs_->AllocateOutput();
    output_min_ = dut_min_->AllocateOutput();

    // Allocate storage for derivatives.
    derivatives_abs_ = dut_abs_->AllocateTimeDerivatives();
    derivatives_min_ = dut_min_->AllocateTimeDerivatives();

    // Fix unused input ports.
    const Vector3<double> input_abs(0.0, 0.0, 0.0);
    const double input_min = 0.0;
    dut_abs_->get_input_port(0).FixValue(context_abs_.get(), input_abs);
    dut_min_->get_input_port(0).FixValue(context_min_.get(), input_min);
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
  const systems::ContinuousState<double>& v = context_abs_->
                                                get_continuous_state();
  dut_abs_->CalcOutput(*context_abs_, output_abs_.get());
  for (int i = 0; i < v.size(); ++i)
    EXPECT_EQ(v[i], output_abs_->get_vector_data(0)->value()(i));
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
  BeadOnAWire<double>::ArcLength s;
  s = M_PI / 3.0;
  auto v = dut_abs_->get_pfunction_output(dut_abs_->helix_function(s));
  const double svalue = s.value().value();
  EXPECT_NEAR(v(0), std::cos(svalue), tol);
  EXPECT_NEAR(v(1), std::sin(svalue), tol);
  EXPECT_NEAR(v(2), svalue, tol);

  // Compute the derivative at pi/3.
  s.derivatives()(0) = 1.0;
  s.value().derivatives()(0) = 1.0;
  const auto w = dut_abs_->get_pfunction_first_derivative(
      dut_abs_->helix_function(s));
  EXPECT_NEAR(w(0), -std::sin(svalue), tol);
  EXPECT_NEAR(w(1), std::cos(svalue), tol);
  EXPECT_NEAR(w(2), 1.0, tol);

  // Compute the second derivative at pi/3.
  const auto w2 = dut_abs_->get_pfunction_second_derivative(
      dut_abs_->helix_function(s));
  EXPECT_NEAR(w2(0), -std::cos(svalue), tol);
  EXPECT_NEAR(w2(1), -std::sin(svalue), tol);
  EXPECT_NEAR(w2(2), 0.0, tol);
}

// Tests that the inverse of the helix parameter function produces the
// values and derivatives we expect.
TEST_F(BeadOnAWireTest, InverseHelix) {
  // Set small tolerance value.
  const double tol = std::numeric_limits<double>::epsilon() * 10.0;

  // Compute the value at pi/3.
  const double test_value = M_PI / 3.0;
  BeadOnAWire<double>::ArcLength s;
  s = test_value;
  auto v = dut_abs_->helix_function(s);
  auto sprime = dut_abs_->inverse_helix_function(v);
  EXPECT_NEAR(dut_abs_->get_inv_pfunction_output(s),
              dut_abs_->get_inv_pfunction_output(sprime), tol);

  // Compute the derivative of the inverse helix function.
  s.derivatives()(0) = 1.0;
  s.value().derivatives()(0) = 1.0;
  auto wprime = dut_abs_->inverse_helix_function(dut_abs_->
      helix_function(s));
  const double candidate_value = dut_abs_->get_inv_pfunction_first_derivative(
      wprime);

  // Apply numerical differentiation to the inverse helix function. We
  // do this by computing the output:
  // f⁻¹(f(s+ds)) - f⁻¹(f(s))
  // ------------------------
  //            ds
  const double ds = std::numeric_limits<double>::epsilon();
  double fprime = dut_abs_->get_inv_pfunction_output(
      dut_abs_->inverse_helix_function(dut_abs_->helix_function(s + ds)));
  double f = dut_abs_->get_inv_pfunction_output(
      dut_abs_->inverse_helix_function(dut_abs_->helix_function(s)));
  const double num_value = (fprime - f) / ds;
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
  Eigen::Matrix<BeadOnAWire<double>::ArcLength, 3, 1> xx;
  xx(0).value() = x;
  xx(1).value() = y;
  xx(2).value() = z;
  xx(0).value().derivatives()(0) = xdot;
  xx(1).value().derivatives()(0) = ydot;
  xx(2).value().derivatives()(0) = zdot;
  double tprime = dut_abs_->get_inv_pfunction_first_derivative(
      dut_abs_->inverse_helix_function(xx));
  EXPECT_NEAR(inv_helix_dot, tprime, tol);
}

// A parametric wire function that allows the bead to move along x² - y² = 0
// (which is orthogonal to gravity).
static Eigen::Matrix<BeadOnAWire<double>::ArcLength, 3, 1>
  horz_line_function(const BeadOnAWire<double>::ArcLength& s) {
  return Vector3<BeadOnAWire<double>::ArcLength>(s*s, s*s, s*0);
}

// The inverse of the parametric wire function that allows the bead to move
// along x² - y² = 0 (which is orthogonal to gravity).
static BeadOnAWire<double>::ArcLength inverse_horz_line_function(
        const Vector3<BeadOnAWire<double>::ArcLength>& v) {
  using std::sqrt;
  return (sqrt(v(1))+sqrt(v(0)))/2;
}

// A parametric wire function that allows the bead to along the z-axis (i.e.,
// parallel to gravity).
static Eigen::Matrix<BeadOnAWire<double>::ArcLength, 3, 1>
  vert_line_function(const BeadOnAWire<double>::ArcLength& s) {
  return Vector3<BeadOnAWire<double>::ArcLength>(s*0, s*0, s);
}

// The inverse of the parametric wire function that allows the bead to along
// the z-axis.
static BeadOnAWire<double>::ArcLength inverse_vert_line_function(
        const Vector3<BeadOnAWire<double>::ArcLength>& v) {
  return v(2);
}


// Tests the constraint function evaluation using the helix function.
TEST_F(BeadOnAWireTest, ConstraintFunctionEval) {
  // Use the vertical wire function, which allows us to determine error
  // precisely.
  dut_abs_->reset_wire_parameter_functions(&vert_line_function,
                                           &inverse_vert_line_function);

  // Put the bead directly onto the wire.
  systems::ContinuousState<double>& xc =
      context_abs_->get_mutable_continuous_state();
  const double s = 1.0;
  xc[0] = 0.0;
  xc[1] = 0.0;
  xc[2] = s;

  // Verify that the constraint error is effectively zero.
  const double tol = std::numeric_limits<double>::epsilon() * 10;
  EXPECT_NEAR(dut_abs_->EvalConstraintEquations(*context_abs_).norm(),
              0.0, tol);

  // Move the bead off of the wire and verify the expected error.
  const double err = 1.0;
  xc[0] += err;
  EXPECT_LE(dut_abs_->EvalConstraintEquations(*context_abs_).norm() - err, tol);
}

// Tests the evaluation of the time derivative of the constraint functions
// using the helix function.
TEST_F(BeadOnAWireTest, ConstraintDotFunctionEval) {
  // Put the bead directly onto the wire and make its velocity such that
  // it is not instantaneously leaving the wire.
  systems::ContinuousState<double>& xc =
      context_abs_->get_mutable_continuous_state();
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

// Verify that the minimal coordinate dynamics computations are reasonable.
TEST_F(BeadOnAWireTest, MinimalCoordinateDeriv) {
  const int n_vars = derivatives_min_->get_vector().size();
  const double tol = std::numeric_limits<double>::epsilon();

  // For horizontal wire function, velocity derivatives should be due to
  // centripetal acceleration only. Centripetal acceleration is v^2/R, where
  // v is the velocity and R is the radius. In this test case, the
  // instantaneous radius (i.e., that for initial s = 1.0) is 1.0 and the
  // initial velocity is 1.0.
  const double ca = 1.0;       // centripetal acceleration
  dut_min_->reset_wire_parameter_functions(&horz_line_function,
                                           &inverse_horz_line_function);
  dut_min_->CalcTimeDerivatives(*context_min_, derivatives_min_.get());
  Eigen::VectorXd deriv = derivatives_min_->get_vector().CopyToVector();
  EXPECT_LT(std::abs(deriv.segment(n_vars/2, n_vars/2).norm() - ca), tol);

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
