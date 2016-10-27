#include "drake/systems/framework/primitives/linear_system.h"
#include "drake/systems/framework/primitives/test/affine_linear_test.h"

using std::make_unique;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace {

class LinearSystemTest : public AffineLinearSystemTest {
 public:
  // Setup an arbitrary LinearSystem.
  LinearSystemTest()
      : AffineLinearSystemTest(0.0, 0.0, 0.0, 0.0) {}

  void SetUp() override { Initialize(); }

  void Initialize() override {
    // Construct the system I/O objects.
    dut_ = make_unique<LinearSystem<double>>(A_, B_, C_, D_);
    dut_->set_name("test_linear_system");
    context_ = dut_->CreateDefaultContext();
    input_vector_ = make_unique<BasicVector<double>>(2 /* size */);
    output_ = dut_->AllocateOutput(*context_);
  }

 protected:
  unique_ptr<LinearSystem<double>> dut_;
};

// Tests that the linear system is correctly setup.
TEST_F(LinearSystemTest, Construction) {
  EXPECT_EQ(1, context_->get_num_input_ports());
  EXPECT_EQ("test_linear_system", dut_->get_name());
  EXPECT_EQ(dut_->GetA(), A_);
  EXPECT_EQ(dut_->GetB(), B_);
  EXPECT_EQ(dut_->GetXDot0(), XDot0_);
  EXPECT_EQ(dut_->GetC(), C_);
  EXPECT_EQ(dut_->GetD(), D_);
  EXPECT_EQ(dut_->GetY0(), Y0_);
  EXPECT_EQ(1, dut_->get_num_output_ports());
  EXPECT_EQ(1, dut_->get_num_input_ports());
}

// Tests that the derivatives are correctly computed.
TEST_F(LinearSystemTest, Derivatives) {
  Eigen::Vector2d u(1, 4);
  SetInput(u);

  Eigen::Vector2d x(0.1, 0.25);
  SetState(x);

  derivatives_ = dut_->AllocateTimeDerivatives();
  EXPECT_NE(derivatives_, nullptr);
  dut_->EvalTimeDerivatives(*context_, derivatives_.get());

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
  SetState(x);

  dut_->EvalOutput(*context_, output_.get());

  Eigen::VectorXd expected_output(2);

  expected_output = C_ * x + D_ * u;

  EXPECT_EQ(
      expected_output,
      output_->get_port(0).get_vector_data<double>()->CopyToVector());
}

}  // namespace
}  // namespace systems
}  // namespace drake
