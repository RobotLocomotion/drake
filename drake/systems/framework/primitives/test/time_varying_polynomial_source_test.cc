#include "drake/systems/framework/primitives/time_varying_polynomial_source.h"

#include <memory>
#include <stdexcept>
#include <string>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/context.h"

#include "gtest/gtest.h"

using Eigen::Matrix;
using Eigen::MatrixXd;
using std::make_unique;

namespace drake {
namespace systems {
namespace {

class TimeVaryingPolynomialSourceTest : public ::testing::Test {
 protected:
  TimeVaryingPolynomialSourceTest() : kppTraj(MatrixXd::Constant(2, 1, 1.5)) {
  }

  void SetUp() override {
    source_ = make_unique<TimeVaryingPolynomialSource<double>>(kppTraj);
    context_ = source_->CreateDefaultContext();
    output_ = source_->AllocateOutput(*context_);
    input_ = make_unique<BasicVector<double>>(3 /* length */);
  }

  static std::unique_ptr<FreestandingInputPort> MakeInput(
      std::unique_ptr<BasicVector<double>> data) {
    return make_unique<FreestandingInputPort>(std::move(data));
  }

  const PiecewisePolynomial<double> kppTraj;
  std::unique_ptr<System<double>> source_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<BasicVector<double>> input_;
};

TEST_F(TimeVaryingPolynomialSourceTest, OutputTest) {
  ASSERT_EQ(0, context_->get_num_input_ports());
  ASSERT_EQ(1, output_->get_num_ports());

  const double kTestTime = 1.75;
  context_->set_time(kTestTime);

  source_->EvalOutput(*context_, output_.get());

  const BasicVector<double>* output_vector = output_->get_vector_data(0);
  ASSERT_NE(nullptr, output_vector);

  Eigen::VectorXd pp_value = kppTraj.value(kTestTime);
  EXPECT_EQ(pp_value, output_vector->get_value());
}

// Tests that inputs cannot be set for a ConstantVectorSource.
TEST_F(TimeVaryingPolynomialSourceTest, ShouldNotBePossibleToConnectInputs) {
  EXPECT_THROW(context_->SetInputPort(0, MakeInput(std::move(input_))),
               std::out_of_range);
}

// Tests that ConstantVectorSource allocates no state variables in the context_.
TEST_F(TimeVaryingPolynomialSourceTest, ConstantVectorSourceIsStateless) {
  EXPECT_EQ(nullptr, context_->get_continuous_state());
}

}  // namespace
}  // namespace systems
}  // namespace drake
