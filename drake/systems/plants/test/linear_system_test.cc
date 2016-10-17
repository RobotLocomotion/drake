#include "drake/systems/plants/affine_linear_system/linear_system_plant.h"

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


class LinearSystemTest : public ::testing::Test {
 public:
  // Setup an arbitrary LinearSystem.
  LinearSystemTest() :
      kA(make_matrix(1.5, 2.7, 3.5, -4.9)),
      kB(make_matrix(4.9, -5.1, 6.8, 7.2)),
      kC(make_matrix(1.1, 2.5, -3.8, 4.6)),
      kD(make_matrix(4.1, 5.6, -6.3, 7.7)) {
  }

  void SetUp() override { Initialize(); }

  void Initialize() {
    // Construct the system I/O objects.
    system_ = make_unique<LinearSystemPlant<double>>(kA, kB, kC, kD);
    system_->set_name("test_linear_system");
    context_ = system_->CreateDefaultContext();
    input_vector_  = make_unique<BasicVector<double>>(2 /* size */);

    Eigen::Vector2d output_eigen_vector(2 /* size */);
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
  unique_ptr<LinearSystemPlant<double>> system_;
  unique_ptr<Context<double>> context_;
  LeafSystemOutput<double> system_output_;

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

// Tests that the linear system is setup correctly.
TEST_F(LinearSystemTest, Construction) {
  EXPECT_EQ(1, context_->get_num_input_ports());
  EXPECT_EQ("test_linear_system", system_->get_name());
  EXPECT_EQ(system_->GetA(), GetA());
  EXPECT_EQ(system_->GetB(), GetB());
  EXPECT_EQ(system_->GetC(), GetC());
  EXPECT_EQ(system_->GetD(), GetD());
  EXPECT_EQ(system_->GetXDot0(), GetXDot0());
  EXPECT_EQ(system_->GetY0(), GetY0());
  EXPECT_EQ(system_->GetXDot0(), Eigen::VectorXd::Zero(2, 1));
  EXPECT_EQ(system_->GetY0(), Eigen::VectorXd::Zero(2, 1));
  EXPECT_EQ(1, system_->get_num_output_ports());
  EXPECT_EQ(1, system_->get_num_input_ports());
}

}  // namespace
}  // namespace systems
}  // namespace drake
