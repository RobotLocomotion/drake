#include "drake/systems/framework/primitives/matrix_gain.h"

#include "drake/systems/framework/primitives/test/affine_linear_test.h"

using std::make_unique;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace {

class MatrixGainTest : public AffineLinearSystemTest {
 public:
  // Setup an arbitrary MatrixGain system.
  MatrixGainTest()
      : AffineLinearSystemTest(0.0, 0.0, 0.0, 0.0) {}

  void Initialize() override {
    // Construct the system I/O objects.
    dut_ = make_unique<MatrixGain<double>>(D_);
    dut_->set_name("test_matrix_gain_system");
    context_ = dut_->CreateDefaultContext();
    input_vector_ = make_unique<BasicVector<double>>(2 /* size */);
    system_output_ = dut_->AllocateOutput(*context_);
  }

 protected:
  // MatrixGain systems have no state variables.
  static const int kNumStates{0};

  // The Device Under Test (DUT) is a MatrixGain<double> system.
  unique_ptr<MatrixGain<double>> dut_;
};

// Tests that the MatrixGain system is correctly setup.
TEST_F(MatrixGainTest, Construction) {
  EXPECT_EQ(context_->get_num_input_ports(), 1);
  EXPECT_EQ(dut_->get_name(), "test_matrix_gain_system");
  EXPECT_EQ(dut_->A(), MatrixX<double>::Zero(kNumStates, kNumStates));
  EXPECT_EQ(dut_->B(), MatrixX<double>::Zero(kNumStates, D_.cols()));
  EXPECT_EQ(dut_->xDot0(), Eigen::VectorXd::Zero(kNumStates));
  EXPECT_EQ(dut_->C(), MatrixX<double>::Zero(D_.rows(), kNumStates));
  EXPECT_EQ(dut_->D(), D_);
  EXPECT_EQ(dut_->y0(), Eigen::VectorXd::Zero(2));
  EXPECT_EQ(dut_->get_num_output_ports(), 1);
  EXPECT_EQ(dut_->get_num_input_ports(), 1);
}

// Tests that the derivatives are correctly computed.
TEST_F(MatrixGainTest, Derivatives) {
  // Input vector `u` can be any value since the system derivatives are not
  // dependent on it.
  Eigen::VectorXd u = VectorX<double>::Zero(D_.cols());
  SetInput(u);

  derivatives_ = dut_->AllocateTimeDerivatives();
  EXPECT_NE(derivatives_, nullptr);
  dut_->EvalTimeDerivatives(*context_, derivatives_.get());

  // We expect the derivatives to be a vector of length zero.
  Eigen::VectorXd expected_derivatives =
      VectorX<double>::Zero(kNumStates);

  EXPECT_EQ(derivatives_->get_vector().CopyToVector(), expected_derivatives);
}

// Tests that the outputs are correctly computed.
TEST_F(MatrixGainTest, Output) {
  // Sets the input vector `u`.
  Eigen::Vector2d u(2.17, 5.99);
  SetInput(u);

  dut_->EvalOutput(*context_, system_output_.get());

  Eigen::VectorXd expected_output(2);
  expected_output = D_ * u;

  EXPECT_EQ(system_output_->get_vector_data(0)->get_value(), expected_output);
}

}  // namespace
}  // namespace systems
}  // namespace drake
