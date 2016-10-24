#include "drake/systems/framework/primitives/mimo_gain.h"
#include "drake/systems/framework/primitives/test/affine_linear_test.h"

using std::make_unique;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace {

class MimoGainTest : public AffineLinearSystemTest {
 public:
  // Setup an arbitrary LinearSystem.
  MimoGainTest()
      : AffineLinearSystemTest(0.0, 0.0, 0.0, 0.0) {}

  void SetUp() override { Initialize(); }

  void Initialize() override {
    // Construct the system I/O objects.
    system_ = make_unique<MimoGain<double>>(D_);
    system_->set_name("test_mimo_gain_system");
    context_ = system_->CreateDefaultContext();
    input_vector_ = make_unique<BasicVector<double>>(2 /* size */);

    Eigen::Vector2d output_eigen_vector(2 /* size */);
    auto output_vector = make_unique<BasicVector<double>>(output_eigen_vector);
    auto output_value = make_unique<VectorValue<double>>(move(output_vector));

    system_output_.add_port(move(output_value));
  }

 protected:
  const int kNumStates_{0};  // MimoGain systems have no state variables.
  unique_ptr<LinearSystemPlant<double>> system_;
};

// Tests that the linear system is correctly setup.
TEST_F(MimoGainTest, Construction) {
  EXPECT_EQ(1, context_->get_num_input_ports());
  EXPECT_EQ("test_mimo_gain_system", system_->get_name());
  EXPECT_EQ(system_->GetA(), MatrixX<double>::Zero(kNumStates_, kNumStates_));
  EXPECT_EQ(system_->GetB(), MatrixX<double>::Zero(kNumStates_, D_.cols()));
  EXPECT_EQ(system_->GetXDot0(), Eigen::VectorXd::Zero(kNumStates_));
  EXPECT_EQ(system_->GetC(), MatrixX<double>::Zero(D_.rows(), kNumStates_));
  EXPECT_EQ(system_->GetD(), D_);
  EXPECT_EQ(system_->GetY0(), Eigen::VectorXd::Zero(2));
  EXPECT_EQ(1, system_->get_num_output_ports());
  EXPECT_EQ(1, system_->get_num_input_ports());
}

// Tests that the derivatives are correctly computed.
TEST_F(MimoGainTest, Derivatives) {
  // Input vector `u` can be any value since the system derivatives are not
  // dependent on it.
  Eigen::VectorXd u = VectorX<double>::Zero(kNumStates_);
  SetInput(u);

  derivatives_ = system_->AllocateTimeDerivatives();
  EXPECT_NE(derivatives_, nullptr);
  system_->EvalTimeDerivatives(*context_, derivatives_.get());

  // We expect the derivatives to be a vector of length zero.
  Eigen::VectorXd expected_derivatives =
      VectorX<double>::Zero(kNumStates_);

  EXPECT_EQ(expected_derivatives, derivatives_->get_vector().CopyToVector());
}

// Tests that the outputs are correctly computed.
TEST_F(MimoGainTest, Output) {
  // Sets the input vector `u`.
  Eigen::Vector2d u(2.17, 5.99);
  SetInput(u);

  system_->EvalOutput(*context_, &system_output_);

  Eigen::VectorXd expected_output(2);
  expected_output = D_ * u;

  EXPECT_EQ(expected_output,
      system_output_.get_port(0).get_vector_data<double>()->CopyToVector());
}

}  // namespace
}  // namespace systems
}  // namespace drake
