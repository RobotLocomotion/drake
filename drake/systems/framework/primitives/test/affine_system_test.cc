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

  EXPECT_TRUE(CompareMatrices(
      expected_derivatives, derivatives_->get_vector().CopyToVector(), 1e-10));
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

  EXPECT_TRUE(CompareMatrices(
      expected_output, system_output_->get_vector_data(0)->get_value(), 1e-10));
}

class FeedthroughAffineSystemTest : public ::testing::Test {
 public:
  void SetDCornerElement(double d_1_1_element) {
    d_1_1_element_ = d_1_1_element;
  }

  void InitialiseSystem() {
    // Setup an arbitrary AffineSystem which is feedthrough if and only if
    // the elements of D matrix are exactly 0.
    Eigen::MatrixXd A_(
        AffineLinearSystemTest::make_2x2_matrix(1.5, 2.7, 3.5, -4.9));
    Eigen::MatrixXd B_(
        AffineLinearSystemTest::make_2x2_matrix(4.9, -5.1, 6.8, 7.2));
    Eigen::VectorXd xDot0_(AffineLinearSystemTest::make_2x1_vector(0, 0));
    Eigen::MatrixXd C_(
        AffineLinearSystemTest::make_2x2_matrix(1.1, 2.5, -3.8, 4.6));
    Eigen::MatrixXd D_(
        AffineLinearSystemTest::make_2x2_matrix(d_1_1_element_, 0, 0, 0));
    Eigen::VectorXd y0_(AffineLinearSystemTest::make_2x1_vector(0, 0));
    dut_ = make_unique<AffineSystem<double>>(A_, B_, xDot0_, C_, D_, y0_);
    dut_->set_name("test_feedtroughaffine_system");
  }

 protected:
  // The Device Under Test is an AffineSystem<double>.
  unique_ptr<AffineSystem<double>> dut_;
  double d_1_1_element_{0.0};
};

// Tests that the system does not render as a direct feedthrough
// when all elements of D are exactly 0.
TEST_F(FeedthroughAffineSystemTest, NoFeedthroughTest) {
  SetDCornerElement(0.0);
  InitialiseSystem();
  EXPECT_FALSE(dut_->has_any_direct_feedthrough());
}

// Tests that the system renders as a direct feedthrough
// when a single element of D is set to 1e-12.
TEST_F(FeedthroughAffineSystemTest, FeedthroughTest) {
  SetDCornerElement(1e-12);
  InitialiseSystem();
  EXPECT_TRUE(dut_->has_any_direct_feedthrough());
}

}  // namespace
}  // namespace systems
}  // namespace drake
