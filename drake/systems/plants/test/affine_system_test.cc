#include "drake/systems/plants/affine_linear_system/affine_system_plant.h"

#include "gtest/gtest.h"

#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/subvector.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/vector_base.h"

using std::make_unique;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace {

class AffineSystemTest : public ::testing::Test {
 public:
  // Setup an arbitrary AffineSystem.
  AffineSystemTest() :
  kA(make_matrix(1.5, 2.7, 3.5, -4.9)),
  kB(make_matrix(4.9, -5.1, 6.8, 7.2)),
  kC(make_matrix(1.1, 2.5, -3.8, 4.6)),
  kD(make_matrix(4.1, 5.6, -6.3, 7.7)),
  kXDot0(make_vector(-4.5, 6.5)),
  kY0(make_vector(3.5, -7.6)) {
  }

  void SetUp() override { Initialize(); }

  void Initialize() {
    // Construct the system I/O objects.
    system_ = make_unique<AffineSystemPlant<double>>(
        kA, kB, kC, kD, kXDot0, kY0);
    system_->set_name("test_affine_system");
    context_ = system_->CreateDefaultContext();
    system_derivatives_ = system_->AllocateTimeDerivatives();
    input_vector_  = make_unique<BasicVector<double>>(2 /* size */);

    Eigen::Vector2d output_eigen_vector(2);
    auto output_vector = make_unique<BasicVector<double>>(output_eigen_vector);
    auto output_value = make_unique<VectorValue<double>>(move(output_vector));

    system_output_.add_port(move(output_value));
  }

  void SetState(const Eigen::Ref<const VectorX<double>>& x) {
    state_vector_ = make_unique<BasicVector<double>>(x);
    state_ = make_unique<ContinuousState<double>>(move(state_vector_));
    context_->set_continuous_state(std::move(state_));
  }

  void SetInput(const Eigen::Ref<const VectorX<double>>& u) {
    input_vector_->get_mutable_value() << u;
    context_->SetInputPort(0, make_unique<FreestandingInputPort>(
        std::move(input_vector_)));
  }

  // Helper method to create input ports (free standing input ports) that are
  // not connected to any other output port in the system.
  // Used to test standalone systems not part of a Diagram.
  static std::unique_ptr<FreestandingInputPort> MakeInput(
      std::unique_ptr<BasicVector<double>> data) {
    return make_unique<FreestandingInputPort>(std::move(data));
  }

  const MatrixX<double> GetA(void) const { return kA; }
  const MatrixX<double> GetB(void) const { return kB; }
  const MatrixX<double> GetC(void) const { return kC; }
  const MatrixX<double> GetD(void) const { return kD; }
  const VectorX<double> GetXDot0(void) const { return kXDot0; }
  const VectorX<double> GetY0(void) const { return kY0; }

 protected:
  unique_ptr<AffineSystemPlant<double>> system_;
  unique_ptr<Context<double>> context_;
  LeafSystemOutput<double> system_output_;
  unique_ptr<ContinuousState<double>> system_derivatives_;

  unique_ptr<ContinuousState<double>> state_;
  unique_ptr<BasicVector<double>> state_vector_;
  unique_ptr<BasicVector<double>> input_vector_;
  unique_ptr<ContinuousState<double>> derivatives_;

 private:
  const Eigen::MatrixXd kA;
  const Eigen::MatrixXd kB;
  const Eigen::MatrixXd kC;
  const Eigen::MatrixXd kD;
  const Eigen::VectorXd kXDot0;
  const Eigen::VectorXd kY0;

  Eigen::MatrixXd make_matrix(double a, double b, double c, double d) {
    Eigen::MatrixXd m(2, 2);
    m << a, b, c, d;
    return m;
  }

  Eigen::VectorXd make_vector(double a, double b) {
    Eigen::VectorXd v(2);
    v << a, b;
    return v;
  }
};

// Tests that the affine system is setup correctly.
TEST_F(AffineSystemTest, Construction) {
  EXPECT_EQ(1, context_->get_num_input_ports());
  EXPECT_EQ("test_affine_system", system_->get_name());
  EXPECT_EQ(system_->GetA(), GetA());
  EXPECT_EQ(system_->GetB(), GetB());
  EXPECT_EQ(system_->GetC(), GetC());
  EXPECT_EQ(system_->GetD(), GetD());
  EXPECT_EQ(system_->GetXDot0(), GetXDot0());
  EXPECT_EQ(system_->GetY0(), GetY0());
  EXPECT_EQ(1, system_->get_num_output_ports());
  EXPECT_EQ(1, system_->get_num_input_ports());
}

// Tests that the derivatives are correctly computed
TEST_F(AffineSystemTest, Derivatives) {
  Eigen::Vector2d u(2);
  u << 1, 4;
  SetInput(u);

  Eigen::VectorXd x(2);
  x << 0.1, 0.25;
  SetState(x);

  derivatives_ = system_->AllocateTimeDerivatives();
  EXPECT_NE(derivatives_, nullptr);
  system_->EvalTimeDerivatives(*context_, derivatives_.get());

  Eigen::VectorXd expected_derivatives(2);
  expected_derivatives = GetA() * x + GetB() * u +  GetXDot0();

  EXPECT_EQ(expected_derivatives, derivatives_->get_state().CopyToVector());
}

// Tests that the outputs are correctly computed.
TEST_F(AffineSystemTest, Output) {
  // Sets the context's input port.
  Eigen::Vector2d u(2);
  u << 5.6, -10.1;
  SetInput(u);

  // Sets the state
  Eigen::VectorXd x(2);
  x << 0.8, -22.1;
  SetState(x);

  system_->EvalOutput(*context_, &system_output_);

  Eigen::VectorXd expected_output(2);

  expected_output = GetC() * x + GetD() * u +  GetY0();

  EXPECT_EQ(expected_output,
            system_output_.get_port(0).
                get_vector_data<double>()->CopyToVector());
}

}  // namespace
}  // namespace systems
}  // namespace drake
