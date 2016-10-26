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
  AffineSystemTest()
      : AffineLinearSystemTest(-4.5, 6.5, 3.5, -7.6) {
  }

  void SetUp() override { Initialize(); }

  void Initialize() override {
    // Construct the system I/O objects.
    system_ =
        make_unique<AffineSystem<double>>(A_, B_, XDot0_, C_, D_, Y0_);
    system_->set_name("test_affine_system");
    context_ = system_->CreateDefaultContext();
    system_derivatives_ = system_->AllocateTimeDerivatives();
    input_vector_ = make_unique<BasicVector<double>>(2 /* size */);

    Eigen::Vector2d output_eigen_vector(2);
    auto output_vector = make_unique<BasicVector<double>>(output_eigen_vector);
    auto output_value = make_unique<VectorValue<double>>(move(output_vector));

    system_output_.add_port(move(output_value));
  }

 protected:
  unique_ptr<AffineSystem<double>> system_;
};

// Tests that the affine system is correctly setup.
TEST_F(AffineSystemTest, Construction) {
  EXPECT_EQ(1, context_->get_num_input_ports());
  EXPECT_EQ("test_affine_system", system_->get_name());
  EXPECT_EQ(system_->GetA(), A_);
  EXPECT_EQ(system_->GetB(), B_);
  EXPECT_EQ(system_->GetC(), C_);
  EXPECT_EQ(system_->GetD(), D_);
  EXPECT_EQ(system_->GetXDot0(), XDot0_);
  EXPECT_EQ(system_->GetY0(), Y0_);
  EXPECT_EQ(1, system_->get_num_output_ports());
  EXPECT_EQ(1, system_->get_num_input_ports());
}

// Tests that the derivatives are correctly computed.
TEST_F(AffineSystemTest, Derivatives) {
  Eigen::Vector2d u(1, 4);
  SetInput(u);

  Eigen::Vector2d x(0.1, 0.25);
  SetState(x);

  derivatives_ = system_->AllocateTimeDerivatives();
  EXPECT_NE(derivatives_, nullptr);
  system_->EvalTimeDerivatives(*context_, derivatives_.get());

  Eigen::VectorXd expected_derivatives(2);
  expected_derivatives = A_ * x + B_ * u + XDot0_;

  EXPECT_EQ(expected_derivatives, derivatives_->get_vector().CopyToVector());
}

// Tests that the outputs are correctly computed.
TEST_F(AffineSystemTest, Output) {
  // Sets the context's input port.
  Eigen::Vector2d u(5.6, -10.1);
  SetInput(u);

  // Sets the state
  Eigen::Vector2d x(0.8, -22.1);
  SetState(x);

  system_->EvalOutput(*context_, &system_output_);

  Eigen::VectorXd expected_output(2);

  expected_output = C_ * x + D_ * u + Y0_;

  EXPECT_EQ(
      expected_output,
      system_output_.get_port(0).get_vector_data<double>()->CopyToVector());
}

}  // namespace
}  // namespace systems
}  // namespace drake
