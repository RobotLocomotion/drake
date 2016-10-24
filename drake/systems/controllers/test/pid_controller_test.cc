#include "drake/systems/controllers/pid_controller.h"

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"

#include "gtest/gtest.h"

using std::make_unique;

namespace drake {
namespace systems {
namespace {

typedef Eigen::Matrix<double, 3, 1, Eigen::DontAlign> Vector3d;

// TODO(amcastro-tri): Create a diagram with a ConstantVectorSource feeding
// the input of the Gain system.
template <class T>
std::unique_ptr<FreestandingInputPort> MakeInput(
    std::unique_ptr<BasicVector<T>> data) {
  return make_unique<FreestandingInputPort>(std::move(data));
}

class PidControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    context_ = controller_.CreateDefaultContext();
    output_ = controller_.AllocateOutput(*context_);
    derivatives_ = controller_.AllocateTimeDerivatives();

    // Error signal input port.
    auto vec0 = std::make_unique<BasicVector<double>>(port_size_);
    vec0->get_mutable_value() << error_signal_;
    context_->SetInputPort(0, MakeInput(std::move(vec0)));

    // Error signal rate input port.
    auto vec1 = std::make_unique<BasicVector<double>>(port_size_);
    vec1->get_mutable_value() << error_rate_signal_;
    context_->SetInputPort(1, MakeInput(std::move(vec1)));
  }

  const int port_size_{3};
  const VectorX<double> kp_{VectorX<double>::Ones(port_size_) * 2.0};
  const VectorX<double> ki_{VectorX<double>::Ones(port_size_) * 3.0};
  const VectorX<double> kd_{VectorX<double>::Ones(port_size_) * 1.0};

  PidController<double> controller_{kp_, ki_, kd_};
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<ContinuousState<double>> derivatives_;
  Vector3d error_signal_{1.0, 2.0, 3.0};
  Vector3d error_rate_signal_{1.3, 0.9, 3.14};
};

// Tests getter methods for controller constants.
TEST_F(PidControllerTest, Getters) {
  ASSERT_EQ(kp_, controller_.get_Kp_vector());
  ASSERT_EQ(ki_, controller_.get_Ki_vector());
  ASSERT_EQ(kd_, controller_.get_Kd_vector());

  EXPECT_NO_THROW(controller_.get_Kp());
  EXPECT_NO_THROW(controller_.get_Ki());
  EXPECT_NO_THROW(controller_.get_Kd());
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

  EXPECT_DEATH(controller.get_Kp(), ".*");
  EXPECT_DEATH(controller.get_Ki(), ".*");
  EXPECT_DEATH(controller.get_Kd(), ".*");
}

// Evaluates the output and asserts correctness.
TEST_F(PidControllerTest, EvalOutput) {
  ASSERT_NE(nullptr, context_);
  ASSERT_NE(nullptr, output_);

  // Initializes the controllers' context to the default value in which the
  // integral is zero and evaluates the output.
  controller_.SetDefaultState(context_.get());
  controller_.EvalOutput(*context_, output_.get());
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
  controller_.EvalOutput(*context_, output_.get());
  EXPECT_EQ(
      (kp_.array() * error_signal_.array() +
       ki_.array() * integral_value.array() +
       kd_.array() * error_rate_signal_.array()).matrix(),
      output_vector->get_value());
}

// Evaluates derivatives and asserts correctness.
TEST_F(PidControllerTest, EvalTimeDerivatives) {
  ASSERT_NE(nullptr, context_);
  ASSERT_NE(nullptr, derivatives_);

  // Initializes the controllers' context to the default value in which the
  // integral is zero and evaluates the derivatives.
  controller_.SetDefaultState(context_.get());

  controller_.EvalTimeDerivatives(*context_, derivatives_.get());
  ASSERT_EQ(3, derivatives_->size());
  ASSERT_EQ(0, derivatives_->get_generalized_position().size());
  ASSERT_EQ(0, derivatives_->get_generalized_velocity().size());
  ASSERT_EQ(3, derivatives_->get_misc_continuous_state().size());

  // The only state in the PID controller_ is the integral of the input signal.
  // Therefore the time derivative of the state equals the input error signal.
  EXPECT_EQ(error_signal_, derivatives_->CopyToVector());
}

}  // namespace
}  // namespace systems
}  // namespace drake
