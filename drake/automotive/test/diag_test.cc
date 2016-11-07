#include "drake/automotive/diag.h"

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"

#include "gtest/gtest.h"

using std::make_unique;

namespace drake {
namespace automotive {
namespace {

typedef Eigen::Matrix<double, 1, 1, Eigen::DontAlign> Vector3d;

template <class T>
std::unique_ptr<systems::FreestandingInputPort> MakeInput(
    std::unique_ptr<systems::BasicVector<T>> data) {
  return make_unique<systems::FreestandingInputPort>(std::move(data));
}

class DiagTest : public ::testing::Test {
 protected:
  void SetUp() override {
    context_ = controller_.CreateDefaultContext();
    output_ = controller_.AllocateOutput(*context_);
    derivatives_ = controller_.AllocateTimeDerivatives();

    // Error signal input port.
    //auto vec0 = std::make_unique<systems::BasicVector<double>>(port_size_);
    //vec0->get_mutable_value() << error_signal_;
    //context_->SetInputPort(0, MakeInput(std::move(vec0)));

    // Error signal rate input port.
    //auto vec1 = std::make_unique<systems::BasicVector<double>>(port_size_);
    //vec1->get_mutable_value() << error_rate_signal_;
    //context_->SetInputPort(1, MakeInput(std::move(vec1)));
  }

  systems::VectorBase<double>* continuous_state() {
    auto result =
        context_->get_mutable_continuous_state_vector();
    if (result == nullptr) { throw std::bad_cast(); }
    return result;
  }

  const VectorX<double> kp_{VectorX<double>::Ones(1) * 2.0};
  const VectorX<double> ki_{VectorX<double>::Ones(1) * 3.0};
  const VectorX<double> kd_{VectorX<double>::Ones(1) * 1.0};

  Diag<double> controller_{kp_, ki_, kd_, 10.0, 0.0};
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};

TEST_F(DiagTest, Topology) {
  ASSERT_EQ(0, controller_.get_num_input_ports());

  ASSERT_EQ(2, controller_.get_num_output_ports());
  const auto& output_ego_car_pos = controller_.get_output_ports().at(0);
  const auto& output_agent_car_pos = controller_.get_output_ports().at(0);
  EXPECT_EQ(systems::kVectorValued, output_ego_car_pos.get_data_type());
  EXPECT_EQ(systems::kVectorValued, output_agent_car_pos.get_data_type());
  EXPECT_EQ(systems::kOutputPort, output_ego_car_pos.get_face());
  EXPECT_EQ(systems::kOutputPort, output_agent_car_pos.get_face());
  EXPECT_EQ(2, output_ego_car_pos.get_size());
  EXPECT_EQ(2, output_agent_car_pos.get_size());
  EXPECT_EQ(systems::kContinuousSampling, output_ego_car_pos.get_sampling());
  EXPECT_EQ(systems::kContinuousSampling, output_agent_car_pos.get_sampling());
}

// Evaluates the output.
TEST_F(DiagTest, EvalOutput) {
  ASSERT_NE(nullptr, context_);
  ASSERT_NE(nullptr, output_);

  // Initializes the controllers' context to the default value in which the
  // integral is zero and evaluates the output.
  controller_.SetDefaultState(context_.get());
  controller_.EvalOutput(*context_, output_.get());
  //controller_.EvalTimeDerivatives(*context_, derivatives_.get());
  const systems::BasicVector<double>* output_ego_pos
    = output_->get_vector_data(0);
  const systems::BasicVector<double>* output_agent_pos
    = output_->get_vector_data(1);

  // Starting state and output is all zeros.
  controller_.EvalOutput(*context_, output_.get());
  EXPECT_EQ(0.0, output_ego_pos->GetAtIndex(0));
  EXPECT_EQ(0.0, output_agent_pos->GetAtIndex(0));

  // Set a new trial state.
  continuous_state()->SetAtIndex(0, 1.0);
  continuous_state()->SetAtIndex(1, 2.0);
  continuous_state()->SetAtIndex(2, 3.0);
  continuous_state()->SetAtIndex(3, 4.0);
  controller_.EvalOutput(*context_, output_.get());
  EXPECT_EQ(3.0, output_ego_pos->GetAtIndex(0));
  EXPECT_EQ(1.0, output_agent_pos->GetAtIndex(0));
}

// Evaluate the derivatives.
TEST_F(DiagTest, EvalTimeDerivatives) {
  ASSERT_NE(nullptr, context_);
  ASSERT_NE(nullptr, derivatives_);

  // Initializes the controllers' context to the default value in which the
  // integral is zero and evaluates the derivatives.
  controller_.SetDefaultState(context_.get());
  controller_.EvalOutput(*context_, output_.get());
  controller_.EvalTimeDerivatives(*context_, derivatives_.get());

  EXPECT_EQ(0.0, continuous_state()->GetAtIndex(0));
  EXPECT_EQ(0.0, continuous_state()->GetAtIndex(1));
  EXPECT_NEAR(0.95061, continuous_state()->GetAtIndex(2), 1e-6);
  EXPECT_EQ(0.0, continuous_state()->GetAtIndex(3));
}

}  // namespace
}  // namespace systems
}  // namespace drake
