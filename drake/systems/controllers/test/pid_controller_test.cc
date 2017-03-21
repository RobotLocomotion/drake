#include "drake/systems/controllers/pid_controller.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/input_port_value.h"

using std::make_unique;

namespace drake {
namespace systems {
namespace {

typedef Eigen::Matrix<double, 3, 1, Eigen::DontAlign> Vector3d;

class PidControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    context_ = controller_.CreateDefaultContext();
    output_ = controller_.AllocateOutput(*context_);
    derivatives_ = controller_.AllocateTimeDerivatives();

    // State:
    auto vec0 = std::make_unique<BasicVector<double>>(port_size_ * 2);
    vec0->get_mutable_value().setZero();
    context_->FixInputPort(0, std::move(vec0));

    // Desired state:
    auto vec1 = std::make_unique<BasicVector<double>>(port_size_ * 2);
    vec1->get_mutable_value() << error_signal_, error_rate_signal_;
    context_->FixInputPort(1, std::move(vec1));
  }

  const int port_size_{3};
  const VectorX<double> kp_{VectorX<double>::Ones(port_size_) * 2.0};
  const VectorX<double> ki_{VectorX<double>::Ones(port_size_) * 3.0};
  const VectorX<double> kd_{VectorX<double>::Ones(port_size_) * 1.0};

  PidController<double> controller_{kp_, ki_, kd_};
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<ContinuousState<double>> derivatives_;
  // Error = estimated - desired.
  Vector3d error_signal_{1.0, 2.0, 3.0};
  Vector3d error_rate_signal_{1.3, 0.9, 3.14};
};

// Tests getter methods for controller constants.
TEST_F(PidControllerTest, Getters) {
  ASSERT_EQ(kp_, controller_.get_Kp_vector());
  ASSERT_EQ(ki_, controller_.get_Ki_vector());
  ASSERT_EQ(kd_, controller_.get_Kd_vector());

  EXPECT_NO_THROW(controller_.get_Kp_singleton());
  EXPECT_NO_THROW(controller_.get_Ki_singleton());
  EXPECT_NO_THROW(controller_.get_Kd_singleton());
}

TEST_F(PidControllerTest, GetterVectors) {
  const Eigen::Vector2d kp{1.0, 2.0};
  const Eigen::Vector2d ki{1.0, 2.0};
  const Eigen::Vector2d kd{1.0, 2.0};
  PidController<double> controller{kp, ki, kd};

  EXPECT_NO_THROW(controller.get_Kp_vector());
  EXPECT_NO_THROW(controller.get_Ki_vector());
  EXPECT_NO_THROW(controller.get_Kd_vector());

  ASSERT_EQ(kp, controller.get_Kp_vector());
  ASSERT_EQ(ki, controller.get_Ki_vector());
  ASSERT_EQ(kd, controller.get_Kd_vector());

  EXPECT_DEATH(controller.get_Kp_singleton(), ".*");
  EXPECT_DEATH(controller.get_Ki_singleton(), ".*");
  EXPECT_DEATH(controller.get_Kd_singleton(), ".*");
}

TEST_F(PidControllerTest, Graphviz) {
  const std::string dot = controller_.GetGraphvizString();
  EXPECT_NE(std::string::npos, dot.find(
      "label=\"PID Controller | { {<u0> q |<u1> q_d} |<y0> y}\"")) << dot;
}

// Evaluates the output and asserts correctness.
TEST_F(PidControllerTest, CalcOutput) {
  ASSERT_NE(nullptr, context_);
  ASSERT_NE(nullptr, output_);

  // Evaluates the output.
  controller_.CalcOutput(*context_, output_.get());
  ASSERT_EQ(1, output_->get_num_ports());
  const BasicVector<double>* output_vector = output_->get_vector_data(0);
  EXPECT_EQ(3, output_vector->size());
  EXPECT_EQ((kp_.array() * error_signal_.array() +
            kd_.array() * error_rate_signal_.array()).matrix(),
            output_vector->get_value());

  // Initializes the integral to a non-zero value. A more interesting example.
  VectorX<double> integral_value(port_size_);
  integral_value << 3.0, 2.0, 1.0;
  controller_.set_integral_value(context_.get(), integral_value);
  controller_.CalcOutput(*context_, output_.get());
  EXPECT_EQ(
      (kp_.array() * error_signal_.array() +
       ki_.array() * integral_value.array() +
       kd_.array() * error_rate_signal_.array()).matrix(),
      output_vector->get_value());
}

// Evaluates derivatives and asserts correctness.
TEST_F(PidControllerTest, CalcTimeDerivatives) {
  ASSERT_NE(nullptr, context_);
  ASSERT_NE(nullptr, derivatives_);

  // Evaluates the derivatives.
  controller_.CalcTimeDerivatives(*context_, derivatives_.get());
  ASSERT_EQ(3, derivatives_->size());
  ASSERT_EQ(0, derivatives_->get_generalized_position().size());
  ASSERT_EQ(0, derivatives_->get_generalized_velocity().size());
  ASSERT_EQ(3, derivatives_->get_misc_continuous_state().size());

  // The only state in the PID controller_ is the integral of the input signal.
  // Therefore the time derivative of the state equals the input error signal.
  // TODO(siyuanfeng): need to get rid of the - once we switch the integrator
  // to be int(q_d - q), right not it's (q - q_d). so the derivative is flipped.
  EXPECT_EQ(error_signal_, -derivatives_->CopyToVector());
}

}  // namespace
}  // namespace systems
}  // namespace drake
