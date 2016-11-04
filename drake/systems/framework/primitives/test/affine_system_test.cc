#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/framework/primitives/affine_system.h"
#include "drake/systems/framework/primitives/test/affine_linear_test.h"

using std::make_unique;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace {

class AffineSystemTest : public AffineLinearSystemTest {
 public:
  // Setup an arbitrary AffineSystem.
  AffineSystemTest() : AffineLinearSystemTest(-4.5, 6.5, 3.5, -7.6) {}

  void Initialize() override {
    // Construct the system I/O objects.
    dut_ = make_unique<AffineSystem<double>>(A_, B_, xDot0_, C_, D_, y0_);
    dut_->set_name("test_affine_system");
    context_ = dut_->CreateDefaultContext();
    input_vector_ = make_unique<BasicVector<double>>(2 /* size */);
    system_output_ = dut_->AllocateOutput(*context_);
    state_ = context_->get_mutable_continuous_state();
    derivatives_ = dut_->AllocateTimeDerivatives();
  }

 protected:
  // The Device Under Test is an AffineSystem<double>.
  unique_ptr<AffineSystem<double>> dut_;
};

// Tests that the affine system is correctly setup.
TEST_F(AffineSystemTest, Construction) {
  EXPECT_EQ(1, context_->get_num_input_ports());
  EXPECT_EQ("test_affine_system", dut_->get_name());
  EXPECT_EQ(dut_->A(), A_);
  EXPECT_EQ(dut_->B(), B_);
  EXPECT_EQ(dut_->C(), C_);
  EXPECT_EQ(dut_->D(), D_);
  EXPECT_EQ(dut_->xDot0(), xDot0_);
  EXPECT_EQ(dut_->y0(), y0_);
  EXPECT_EQ(dut_->get_num_output_ports(), 1);
  EXPECT_EQ(dut_->get_num_input_ports(), 1);
}

// Tests that the derivatives are correctly computed.
TEST_F(AffineSystemTest, Derivatives) {
  Eigen::Vector2d u(1, 4);
  SetInput(u);

  Eigen::Vector2d x(0.1, 0.25);
  state_->SetFromVector(x);

  EXPECT_NE(derivatives_, nullptr);
  dut_->EvalTimeDerivatives(*context_, derivatives_.get());

  Eigen::VectorXd expected_derivatives(2);
  expected_derivatives = A_ * x + B_ * u + xDot0_;

  EXPECT_EQ(expected_derivatives, derivatives_->get_vector().CopyToVector());
}

// Tests that the outputs are correctly computed.
TEST_F(AffineSystemTest, Output) {
  // Sets the context's input port.
  Eigen::Vector2d u(5.6, -10.1);
  SetInput(u);

  // Sets the state.
  Eigen::Vector2d x(0.8, -22.1);
  state_->SetFromVector(x);

  dut_->EvalOutput(*context_, system_output_.get());

  Eigen::VectorXd expected_output(2);

  expected_output = C_ * x + D_ * u + y0_;

  EXPECT_EQ(expected_output, system_output_->get_vector_data(0)->get_value());
}

// Test that linearizing and affine system returns an identical affine system
GTEST_TEST(testLinearize, fromAffine) {
  Eigen::Matrix3d A;
  Eigen::Matrix<double, 3, 1> B;
  Eigen::Vector3d xDot0;
  Eigen::Matrix<double, 2, 3> C;
  Eigen::Vector2d D;
  Eigen::Vector2d y0;
  A << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  B << 10, 11, 12;
  xDot0 << 13, 14, 15;
  C << 16, 17, 18, 19, 20, 21;
  D << 22, 23;
  y0 << 24, 25;
  AffineSystem<double> system(A, B, xDot0, C, D, y0);
  auto context = system.CreateDefaultContext();
  Eigen::Vector3d x0;
  x0 << 26, 27, 28;
  context->get_mutable_continuous_state_vector()->SetFromVector(x0);
  double u0 = 29;
  auto u0vec = std::make_unique<BasicVector<double>>(1);
  u0vec->SetAtIndex(0, u0);
  context->SetInputPort(
      0, std::make_unique<FreestandingInputPort>(std::move(u0vec)));
  auto linearized_system = Linearize(system, *context);

  double tol = 1e-10;
  EXPECT_TRUE(CompareMatrices(A, linearized_system->A(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(B, linearized_system->B(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(A * x0 + B * u0 + xDot0,
                              linearized_system->xDot0(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(C, linearized_system->C(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(D, linearized_system->D(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(C * x0 + D * u0 + y0, linearized_system->y0(),
                              tol, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace systems
}  // namespace drake
