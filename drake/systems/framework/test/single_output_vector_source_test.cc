#include "drake/systems/framework/single_output_vector_source.h"

#include <memory>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace systems {
namespace {

const int kSize = 3;

class TestSource : public SingleOutputVectorSource<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestSource)

  TestSource() : SingleOutputVectorSource<double>(kSize) {}

  // N.B. This method signature might be used by many downstream projects.
  // Change it only with good reason and with a deprecation period first.
  void DoCalcVectorOutput(
      const Context<double>& context,
      Eigen::VectorBlock<Eigen::VectorXd>* output) const final {
    *output = Eigen::Vector3d::Ones();
  }
};

class SingleOutputVectorSourceTest : public ::testing::Test {
 protected:
  void SetUp() override {
    source_ = std::make_unique<TestSource>();
    context_ = source_->CreateDefaultContext();
    output_ = source_->AllocateOutput(*context_);
  }

  std::unique_ptr<System<double>> source_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

// Tests that the output is correct.
TEST_F(SingleOutputVectorSourceTest, OutputTest) {
  ASSERT_EQ(context_->get_num_input_ports(), 0);
  ASSERT_EQ(output_->get_num_ports(), 1);

  source_->CalcOutput(*context_, output_.get());

  EXPECT_EQ(output_->get_vector_data(0)->get_value(), Eigen::Vector3d::Ones());
}

// Tests that the state is empty.
TEST_F(SingleOutputVectorSourceTest, IsStateless) {
  EXPECT_EQ(context_->get_continuous_state()->size(), 0);
}

}  // namespace
}  // namespace systems
}  // namespace drake
