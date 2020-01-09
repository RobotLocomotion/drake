#include "drake/systems/controllers/pid_controller.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/fixed_input_port_value.h"

using std::make_unique;

namespace drake {
namespace systems {
namespace controllers {
namespace {

typedef Eigen::Matrix<double, 3, 1, Eigen::DontAlign> Vector3d;

class PidControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    context_ = controller_.CreateDefaultContext();
    output_ = controller_.AllocateOutput();
    derivatives_ = controller_.AllocateTimeDerivatives();

    // State:
    VectorX<double> vec0 = VectorX<double>::Zero(port_size_ * 2);
    controller_.get_input_port(0).FixValue(context_.get(), vec0);

    // Desired state:
    VectorX<double> vec1(port_size_ * 2);
    vec1 << error_signal_, error_rate_signal_;
    controller_.get_input_port(1).FixValue(context_.get(), vec1);
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

TEST_F(PidControllerTest, PortNames) {
  EXPECT_EQ(controller_.GetInputPort("estimated_state").get_index(),
            controller_.get_input_port_estimated_state().get_index());
  EXPECT_EQ(controller_.GetInputPort("desired_state").get_index(),
            controller_.get_input_port_desired_state().get_index());
  EXPECT_EQ(controller_.GetOutputPort("control").get_index(),
            controller_.get_output_port_control().get_index());
}

// Tests getter methods for controller constants.
TEST_F(PidControllerTest, Getters) {
  ASSERT_EQ(kp_, controller_.get_Kp_vector());
  ASSERT_EQ(ki_, controller_.get_Ki_vector());
  ASSERT_EQ(kd_, controller_.get_Kd_vector());

  DRAKE_EXPECT_NO_THROW(controller_.get_Kp_singleton());
  DRAKE_EXPECT_NO_THROW(controller_.get_Ki_singleton());
  DRAKE_EXPECT_NO_THROW(controller_.get_Kd_singleton());
}

TEST_F(PidControllerTest, GetterVectors) {
  const Eigen::Vector2d kp{1.0, 2.0};
  const Eigen::Vector2d ki{1.0, 2.0};
  const Eigen::Vector2d kd{1.0, 2.0};
  PidController<double> controller{kp, ki, kd};

  DRAKE_EXPECT_NO_THROW(controller.get_Kp_vector());
  DRAKE_EXPECT_NO_THROW(controller.get_Ki_vector());
  DRAKE_EXPECT_NO_THROW(controller.get_Kd_vector());

  ASSERT_EQ(kp, controller.get_Kp_vector());
  ASSERT_EQ(ki, controller.get_Ki_vector());
  ASSERT_EQ(kd, controller.get_Kd_vector());

  EXPECT_THROW(controller.get_Kp_singleton(), std::runtime_error);
}

TEST_F(PidControllerTest, GetterVectorKi) {
  const Eigen::Vector2d kp{1.0, 2.0};
  const Eigen::Vector2d ki{1.0, 2.0};
  const Eigen::Vector2d kd{1.0, 2.0};
  PidController<double> controller{kp, ki, kd};

  EXPECT_THROW(controller.get_Ki_singleton(), std::runtime_error);
}

TEST_F(PidControllerTest, GetterVectorKd) {
  const Eigen::Vector2d kp{1.0, 2.0};
  const Eigen::Vector2d ki{1.0, 2.0};
  const Eigen::Vector2d kd{1.0, 2.0};
  PidController<double> controller{kp, ki, kd};

  EXPECT_THROW(controller.get_Kd_singleton(), std::runtime_error);
}

TEST_F(PidControllerTest, Graphviz) {
  const std::string dot = controller_.GetGraphvizString();
  EXPECT_NE(
      std::string::npos,
      dot.find("label=\"PID Controller | { {<u0> x |<u1> x_d} |<y0> y}\""))
      << dot;
}

// Evaluates the output and asserts correctness.
TEST_F(PidControllerTest, CalcOutput) {
  ASSERT_NE(nullptr, context_);
  ASSERT_NE(nullptr, output_);

  // Evaluates the output.
  controller_.CalcOutput(*context_, output_.get());
  ASSERT_EQ(1, output_->num_ports());
  const BasicVector<double>* output_vector = output_->get_vector_data(0);
  EXPECT_EQ(3, output_vector->size());
  EXPECT_EQ((kp_.array() * error_signal_.array() +
             kd_.array() * error_rate_signal_.array())
                .matrix(),
            output_vector->get_value());

  // Initializes the integral to a non-zero value. A more interesting example.
  VectorX<double> integral_value(port_size_);
  integral_value << 3.0, 2.0, 1.0;
  controller_.set_integral_value(context_.get(), integral_value);
  controller_.CalcOutput(*context_, output_.get());
  EXPECT_EQ((kp_.array() * error_signal_.array() +
             ki_.array() * integral_value.array() +
             kd_.array() * error_rate_signal_.array())
                .matrix(),
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

  EXPECT_EQ(error_signal_, derivatives_->CopyToVector());
}

TEST_F(PidControllerTest, DirectFeedthrough) {
  // When the proportional or derivative gain is nonzero, there is direct
  // feedthrough from both the state and error inputs to the output.
  EXPECT_TRUE(controller_.HasAnyDirectFeedthrough());
  EXPECT_TRUE(controller_.HasDirectFeedthrough(0, 0));
  EXPECT_TRUE(controller_.HasDirectFeedthrough(1, 0));

  // When the gains are all zero, there is no direct feedthrough from any
  // input to any output.
  const VectorX<double> zero{VectorX<double>::Zero(port_size_)};
  PidController<double> zero_controller(zero, zero, zero);
  EXPECT_FALSE(zero_controller.HasAnyDirectFeedthrough());
}

TEST_F(PidControllerTest, ToAutoDiff) {
  std::unique_ptr<System<AutoDiffXd>> clone = controller_.ToAutoDiffXd();
  ASSERT_NE(clone, nullptr);
  const auto* const downcast =
      dynamic_cast<PidController<AutoDiffXd>*>(clone.get());
  ASSERT_NE(downcast, nullptr);

  EXPECT_EQ(downcast->get_Kp_vector(), kp_);
  EXPECT_EQ(downcast->get_Ki_vector(), ki_);
  EXPECT_EQ(downcast->get_Kd_vector(), kd_);
}

GTEST_TEST(PidConstructorTest, TestThrow) {
  // Gains size mismatch.
  EXPECT_THROW(PidController<double>(VectorX<double>(2), VectorX<double>(3),
                                     VectorX<double>(3)),
               std::logic_error);
  // State projection row != 2 * |kp|.
  EXPECT_THROW(PidController<double>(MatrixX<double>(5, 2), VectorX<double>(3),
                                     VectorX<double>(3), VectorX<double>(3)),
               std::logic_error);
  // Output projection col != |kp|
  EXPECT_THROW(PidController<double>(MatrixX<double>(6, 2),
                                     MatrixX<double>(3, 2), VectorX<double>(3),
                                     VectorX<double>(3), VectorX<double>(3)),
               std::logic_error);
}

// Tests the full version with non identity P_input and P_output.
GTEST_TEST(PidIOProjectionTest, Test) {
  const int num_full_q = 3;
  const int num_controlled_q = 2;
  const int num_control = 3;

  std::srand(1234);

  const VectorX<double> kp = VectorX<double>::Random(num_controlled_q);
  const VectorX<double> kd = VectorX<double>::Random(num_controlled_q);
  const VectorX<double> ki = VectorX<double>::Random(num_controlled_q);
  MatrixX<double> P_input =
      MatrixX<double>::Random(2 * num_controlled_q, 2 * num_full_q);
  MatrixX<double> P_output =
      MatrixX<double>::Random(num_control, num_controlled_q);

  PidController<double> dut(P_input, P_output, kp, ki, kd);
  auto context = dut.CreateDefaultContext();
  auto output = dut.AllocateOutput();

  VectorX<double> x = VectorX<double>::Random(2 * num_full_q);
  VectorX<double> x_d = VectorX<double>::Random(2 * num_controlled_q);
  VectorX<double> integral = VectorX<double>::Random(num_controlled_q);

  // State:
  dut.get_input_port(0).FixValue(context.get(), x);
  // Desired state:
  dut.get_input_port(1).FixValue(context.get(), x_d);
  // Integral:
  dut.set_integral_value(context.get(), integral);

  dut.CalcOutput(*context, output.get());

  VectorX<double> x_err = x_d - P_input * x;
  auto q_err = x_err.head(num_controlled_q);
  auto v_err = x_err.tail(num_controlled_q);

  VectorX<double> output_expected = (kp.array() * q_err.array()).matrix() +
                                    (kd.array() * v_err.array()).matrix() +
                                    (ki.array() * integral.array()).matrix();
  output_expected = P_output * output_expected;

  EXPECT_TRUE(CompareMatrices(output_expected,
                              output->get_vector_data(0)->get_value(), 1e-12,
                              MatrixCompareType::absolute));
}

}  // namespace
}  // namespace controllers
}  // namespace systems
}  // namespace drake
