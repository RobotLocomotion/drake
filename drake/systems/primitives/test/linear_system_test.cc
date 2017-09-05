#include "drake/systems/primitives/linear_system.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"
#include "drake/systems/primitives/test/affine_linear_test.h"

using std::make_unique;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace {

class LinearSystemTest : public AffineLinearSystemTest {
 public:
  // Setup an arbitrary LinearSystem.
  LinearSystemTest() : AffineLinearSystemTest(0.0, 0.0, 0.0, 0.0) {}

  void Initialize() override {
    // Construct the system I/O objects.
    dut_ = make_unique<LinearSystem<double>>(A_, B_, C_, D_);
    dut_->set_name("test_linear_system");
    context_ = dut_->CreateDefaultContext();
    input_vector_ = make_unique<BasicVector<double>>(2 /* size */);
    system_output_ = dut_->AllocateOutput(*context_);
    state_ = context_->get_mutable_continuous_state();
    derivatives_ = dut_->AllocateTimeDerivatives();
  }

 protected:
  // The Device Under Test (DUT) is a LinearSystem<double>.
  unique_ptr<LinearSystem<double>> dut_;
};

// Tests that the linear system is correctly setup.
TEST_F(LinearSystemTest, Construction) {
  EXPECT_EQ(1, context_->get_num_input_ports());
  EXPECT_EQ("test_linear_system", dut_->get_name());
  EXPECT_EQ(dut_->A(), A_);
  EXPECT_EQ(dut_->B(), B_);
  EXPECT_EQ(dut_->f0(), f0_);
  EXPECT_EQ(dut_->C(), C_);
  EXPECT_EQ(dut_->D(), D_);
  EXPECT_EQ(dut_->y0(), y0_);
  EXPECT_EQ(1, dut_->get_num_output_ports());
  EXPECT_EQ(1, dut_->get_num_input_ports());
}

// Tests that the derivatives are correctly computed.
TEST_F(LinearSystemTest, Derivatives) {
  Eigen::Vector2d u(1, 4);
  SetInput(u);

  Eigen::Vector2d x(0.1, 0.25);
  state_->SetFromVector(x);

  EXPECT_NE(derivatives_, nullptr);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());

  Eigen::VectorXd expected_derivatives(2);
  expected_derivatives = A_ * x + B_ * u;

  EXPECT_EQ(expected_derivatives, derivatives_->get_vector().CopyToVector());
}

// Tests that the outputs are correctly computed.
TEST_F(LinearSystemTest, Output) {
  // Sets the context's input port.
  Eigen::Vector2d u(5.6, -10.1);
  SetInput(u);

  // Sets the state.
  Eigen::Vector2d x(0.8, -22.1);
  state_->SetFromVector(x);

  dut_->CalcOutput(*context_, system_output_.get());

  Eigen::VectorXd expected_output(2);

  expected_output = C_ * x + D_ * u;

  EXPECT_EQ(expected_output, system_output_->get_vector_data(0)->get_value());
}

// Tests converting to different scalar types.
TEST_F(LinearSystemTest, ConvertScalarType) {
  // TODO(jwnimmer-tri) We would prefer that these are true, but right now,
  // LinearSystem does not transmogrify correctly.
  EXPECT_FALSE(is_autodiffxd_convertible(*dut_, [&](const auto& converted) {
    EXPECT_EQ(converted.A(), A_);
    EXPECT_EQ(converted.B(), B_);
    EXPECT_EQ(converted.C(), C_);
    EXPECT_EQ(converted.D(), D_);
  }));
  EXPECT_FALSE(is_symbolic_convertible(*dut_, [&](const auto& converted) {
    EXPECT_EQ(converted.A(), A_);
    EXPECT_EQ(converted.B(), B_);
    EXPECT_EQ(converted.C(), C_);
    EXPECT_EQ(converted.D(), D_);
  }));
}

// Test that linearizing a continuous-time affine system returns the original
// A,B,C,D matrices.
GTEST_TEST(TestLinearize, FromAffine) {
  Eigen::Matrix3d A;
  Eigen::Matrix<double, 3, 1> B;
  Eigen::Vector3d f0;
  Eigen::Matrix<double, 2, 3> C;
  Eigen::Vector2d D;
  Eigen::Vector2d y0;
  A << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  B << 10, 11, 12;
  f0 << 13, 14, 15;
  C << 16, 17, 18, 19, 20, 21;
  D << 22, 23;
  y0 << 24, 25;
  AffineSystem<double> system(A, B, f0, C, D, y0);
  auto context = system.CreateDefaultContext();
  Eigen::Vector3d x0(26, 27, 28);
  context->get_mutable_continuous_state_vector()->SetFromVector(x0);
  double u0 = 29;
  context->FixInputPort(0, Vector1d::Constant(u0));

  // This Context is not an equilibrium point.
  EXPECT_THROW(Linearize(system, *context), std::runtime_error);

  // Set x0 to the actual equilibrium point.
  x0 = A.colPivHouseholderQr().solve(-B * u0 - f0);
  context->get_mutable_continuous_state_vector()->SetFromVector(x0);

  auto linearized_system = Linearize(system, *context);

  double tol = 1e-10;
  EXPECT_TRUE(CompareMatrices(A, linearized_system->A(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(B, linearized_system->B(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(C, linearized_system->C(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(D, linearized_system->D(), tol,
                              MatrixCompareType::absolute));
}

// Test that linearizing a discrete-time affine system returns the original
// A,B,C,D matrices and time period.
GTEST_TEST(TestLinearize, FromDiscreteAffine) {
  Eigen::Matrix3d A;
  Eigen::Matrix<double, 3, 1> B;
  Eigen::Vector3d f0;
  Eigen::Matrix<double, 2, 3> C;
  Eigen::Vector2d D;
  Eigen::Vector2d y0;
  A << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  B << 10, 11, 12;
  f0 << 13, 14, 15;
  C << 16, 17, 18, 19, 20, 21;
  D << 22, 23;
  y0 << 24, 25;
  const double time_period = 0.1;
  AffineSystem<double> discrete_system(A, B, f0, C, D, y0, time_period);
  auto context = discrete_system.CreateDefaultContext();
  Eigen::Vector3d x0(26, 27, 28);
  systems::BasicVector<double>* xd =
      context->get_mutable_discrete_state()->get_mutable_vector();
  xd->SetFromVector(x0);
  double u0 = 29;
  context->FixInputPort(0, Vector1d::Constant(u0));

  // This Context is not an equilibrium point.
  EXPECT_THROW(Linearize(discrete_system, *context), std::runtime_error);

  // Set x0 to the actual equilibrium point.
  Eigen::Matrix3d eye = Eigen::Matrix3d::Identity();
  x0 = (eye - A).colPivHouseholderQr().solve(B * u0 + f0);
  xd->SetFromVector(x0);

  auto linearized_system = Linearize(discrete_system, *context);

  double tol = 1e-10;
  EXPECT_TRUE(CompareMatrices(A, linearized_system->A(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(B, linearized_system->B(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(C, linearized_system->C(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(D, linearized_system->D(), tol,
                              MatrixCompareType::absolute));
  EXPECT_EQ(time_period, linearized_system->time_period());
}

// A trivial system with discrete state that is not bound to a periodic update
// cycle.
class TestNonPeriodicSystem : public LeafSystem<double> {
 public:
  TestNonPeriodicSystem() {
    this->DeclareDiscreteState(1);
    PublishEvent<double> event(Event<double>::TriggerType::kPerStep);
    this->DeclarePerStepEvent(event);
  }

  void DoCalcDiscreteVariableUpdates(
      const Context<double>& context,
      const std::vector<const DiscreteUpdateEvent<double>*>&,
      DiscreteValues<double>* discrete_state) const override {
    (*discrete_state)[0] = context.get_discrete_state(0)->GetAtIndex(0) + 1;
  }
};

// Test that Linearize throws when called on a discrete but non-periodic system.
GTEST_TEST(TestLinearize, ThrowsWithNonPeriodicDiscreteSystem) {
  TestNonPeriodicSystem system;
  auto context = system.CreateDefaultContext();

  EXPECT_THROW(Linearize(system, *context), std::runtime_error);
}

// Test a few simple systems that are known to be controllable (or not).
GTEST_TEST(TestLinearize, Controllability) {
  Eigen::Matrix2d A;
  Eigen::Matrix<double, 2, 1> B;
  Eigen::Matrix<double, 0, 2> C;
  Eigen::Matrix<double, 0, 1> D;

  // Controllable system: x1dot = x2, x2dot = u (aka xddot = u).
  A << 0, 1, 0, 0;
  B << 0, 1;
  LinearSystem<double> sys1(A, B, C, D);

  EXPECT_TRUE(IsControllable(sys1));

  // Uncontrollable system: x1dot = u, x2dot = u.
  A << 0, 0, 0, 0;
  B << 1, 1;
  LinearSystem<double> sys2(A, B, C, D);

  EXPECT_FALSE(IsControllable(sys2));
}

// Test a few simple systems that are known to be observable (or not).
GTEST_TEST(TestLinearize, Observability) {
  Eigen::Matrix2d A;
  Eigen::Matrix<double, 2, 1> B;
  Eigen::Matrix<double, 2, 2> C;
  Eigen::Matrix<double, 2, 1> D;

  // x1dot = x2, x2dot = u (aka  xddot = u).
  A << 0, 1, 0, 0;
  B << 0, 1;
  D << 0, 0;

  // Observable: y = x.
  C << 1, 0, 0, 1;
  LinearSystem<double> sys1(A, B, C, D);
  EXPECT_TRUE(IsObservable(sys1));

  // Unobservable: y = x2;
  LinearSystem<double> sys2(A, B, C.bottomRows(1), D.bottomRows(1));
  EXPECT_FALSE(IsObservable(sys2));

  // Observable: y = x1;
  LinearSystem<double> sys3(A, B, C.topRows(1), D.topRows(1));
  EXPECT_TRUE(IsObservable(sys3));
}

class LinearSystemSymbolicTest : public ::testing::Test {
 public:
  LinearSystemSymbolicTest() = default;
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearSystemSymbolicTest)

 protected:
  void SetUp() override {
    // clang-format off
  A_  << 1, 0, 3,
        -4, 5, 0,
         7, 0, 9;
  B_  << 10, -7,
         12, 0,
         0,  15;
  C_  <<  1, 2,  0,
         -4, 0, -7;
  D_  << -3,  9,
          0, 13;
  x_ << x0_, x1_, x2_;
  u_ << u0_, u1_;
    // clang-format on
  }

  const symbolic::Variable x0_{"x0"};
  const symbolic::Variable x1_{"x1"};
  const symbolic::Variable x2_{"x2"};
  const symbolic::Variable u0_{"u0"};
  const symbolic::Variable u1_{"u1"};
  Eigen::MatrixXd A_{3, 3};
  Eigen::MatrixXd B_{3, 2};
  Eigen::MatrixXd C_{2, 3};
  Eigen::MatrixXd D_{2, 2};
  VectorX<symbolic::Variable> x_{3};
  VectorX<symbolic::Variable> u_{2};
};

TEST_F(LinearSystemSymbolicTest, MakeLinearSystem) {
  // Checks if MakeLinearSystem() parses the arguments and build a
  // system correctly.
  const auto dut = LinearSystem<double>::MakeLinearSystem(
      A_ * x_ + B_ * u_, C_ * x_ + D_ * u_, x_, u_, 10.0);
  EXPECT_EQ(dut->A(), A_);
  EXPECT_EQ(dut->B(), B_);
  EXPECT_EQ(dut->C(), C_);
  EXPECT_EQ(dut->D(), D_);
  EXPECT_EQ(dut->time_period(), 10.0);
}

// Adds quadratic terms to check if we have an exception. Note that we have
// similar testcases in drake/common/test/symbolic_decompose_test.cc file but we
// believe that having redundancy is not bad in testing.
TEST_F(LinearSystemSymbolicTest, MakeLinearSystemException1) {
  VectorX<symbolic::Expression> extra_terms(3);
  // clang-format off
  extra_terms << x0_ * x0_,
                 x1_ * x1_,
                 x2_ * x2_;
  // clang-format on
  EXPECT_THROW(
      LinearSystem<double>::MakeLinearSystem(extra_terms + A_ * x_ + B_ * u_,
                                             C_ * x_ + D_ * u_, x_, u_, 10.0),
      std::runtime_error);
}

// Adds bilinear terms to check if we have an exception.
TEST_F(LinearSystemSymbolicTest, MakeLinearSystemException2) {
  VectorX<symbolic::Expression> extra_terms(3);
  // clang-format off
  extra_terms << x0_ * u0_,
                 x1_ * u1_,
                 x2_ * u0_;
  // clang-format on
  EXPECT_THROW(
      LinearSystem<double>::MakeLinearSystem(extra_terms + A_ * x_ + B_ * u_,
                                             C_ * x_ + D_ * u_, x_, u_, 10.0),
      std::runtime_error);
}

// Adds nonlinear terms to check if we have an exception.
TEST_F(LinearSystemSymbolicTest, MakeLinearSystemException3) {
  VectorX<symbolic::Expression> extra_terms(3);
  // clang-format off
  extra_terms << sin(x0_),
                 cos(x1_),
                 log(u0_);
  // clang-format on
  EXPECT_THROW(
      LinearSystem<double>::MakeLinearSystem(extra_terms + A_ * x_ + B_ * u_,
                                             C_ * x_ + D_ * u_, x_, u_, 10.0),
      std::runtime_error);
}

// Adds constant terms to check if we have an exception.
TEST_F(LinearSystemSymbolicTest, MakeLinearSystemException4) {
  VectorX<symbolic::Expression> extra_terms(3);
  // clang-format off
  extra_terms << -1,
                  1,
                  M_PI;
  // clang-format on
  EXPECT_THROW(
      LinearSystem<double>::MakeLinearSystem(extra_terms + A_ * x_ + B_ * u_,
                                             C_ * x_ + D_ * u_, x_, u_, 10.0),
      std::runtime_error);
}

}  // namespace
}  // namespace systems
}  // namespace drake
