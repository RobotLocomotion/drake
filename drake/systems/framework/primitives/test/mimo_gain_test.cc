#include "drake/systems/framework/primitives/mimo_gain.h"

#include "drake/systems/framework/primitives/test/affine_linear_test.h"

using std::make_unique;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace {

class MimoGainTest : public AffineLinearSystemTest {
 public:
  // Setup an arbitrary MimoGain system.
  MimoGainTest()
      : AffineLinearSystemTest(0.0, 0.0, 0.0, 0.0) {}

  void Initialize() override {
    // Construct the system I/O objects.
    mimo_gain_ = make_unique<MimoGain<double>>(D_);
    mimo_gain_->set_name("test_mimo_gain_system");
    context_ = mimo_gain_->CreateDefaultContext();
    input_vector_ = make_unique<BasicVector<double>>(2 /* size */);
    system_output_ = mimo_gain_->AllocateOutput(*context_);
  }

 protected:
  const int kNumStates{0};  // MimoGain systems have no state variables.
  unique_ptr<MimoGain<double>> mimo_gain_;
};

// Tests that the MimoGain system is correctly setup.
TEST_F(MimoGainTest, Construction) {
  EXPECT_EQ(context_->get_num_input_ports(), 1);
  EXPECT_EQ(mimo_gain_->get_name(), "test_mimo_gain_system");
  EXPECT_EQ(mimo_gain_->GetA(), MatrixX<double>::Zero(kNumStates, kNumStates));
  EXPECT_EQ(mimo_gain_->GetB(), MatrixX<double>::Zero(kNumStates, D_.cols()));
  EXPECT_EQ(mimo_gain_->GetxDot0(), Eigen::VectorXd::Zero(kNumStates));
  EXPECT_EQ(mimo_gain_->GetC(), MatrixX<double>::Zero(D_.rows(), kNumStates));
  EXPECT_EQ(mimo_gain_->GetD(), D_);
  EXPECT_EQ(mimo_gain_->Gety0(), Eigen::VectorXd::Zero(2));
  EXPECT_EQ(mimo_gain_->get_num_output_ports(), 1);
  EXPECT_EQ(mimo_gain_->get_num_input_ports(), 1);
}

// Tests that the derivatives are correctly computed.
TEST_F(MimoGainTest, Derivatives) {
  // Input vector `u` can be any value since the system derivatives are not
  // dependent on it.
  Eigen::VectorXd u = VectorX<double>::Zero(D_.cols());
  SetInput(u);

  derivatives_ = mimo_gain_->AllocateTimeDerivatives();
  EXPECT_NE(derivatives_, nullptr);
  mimo_gain_->EvalTimeDerivatives(*context_, derivatives_.get());

  // We expect the derivatives to be a vector of length zero.
  Eigen::VectorXd expected_derivatives =
      VectorX<double>::Zero(kNumStates);

  EXPECT_EQ(derivatives_->get_vector().CopyToVector(), expected_derivatives);
}

// Tests that the outputs are correctly computed.
TEST_F(MimoGainTest, Output) {
  // Sets the input vector `u`.
  Eigen::Vector2d u(2.17, 5.99);
  SetInput(u);

  mimo_gain_->EvalOutput(*context_, system_output_.get());

  Eigen::VectorXd expected_output(2);
  expected_output = D_ * u;

  EXPECT_EQ(system_output_->get_vector_data(0)->get_value(), expected_output);
}

}  // namespace
}  // namespace systems
}  // namespace drake
