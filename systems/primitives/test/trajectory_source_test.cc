#include "drake/systems/primitives/trajectory_source.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/fixed_input_port_value.h"

using Eigen::Matrix;
using Eigen::MatrixXd;
using std::make_unique;

namespace drake {
namespace systems {

using trajectories::PiecewisePolynomial;

namespace {

class TrajectorySourceTest : public ::testing::Test {
 protected:
  const size_t kDerivativeOrder = 5;

  void BuildSource(PiecewisePolynomial<double> pp) {
    kppTraj_ = make_unique<PiecewisePolynomial<double>>(pp);
    source_ = make_unique<TrajectorySource<double>>(*kppTraj_, kDerivativeOrder,
                                                    true);
    context_ = source_->CreateDefaultContext();
    input_ = make_unique<BasicVector<double>>(3 /* length */);
  }

  std::unique_ptr<PiecewisePolynomial<double>> kppTraj_;
  std::unique_ptr<TrajectorySource<double>> source_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<BasicVector<double>> input_;
};

TEST_F(TrajectorySourceTest, OutputTest) {
  const Polynomiald y = Polynomiald("y");
  const Polynomiald yyyy = (y * y * y * y);
  const std::vector<Polynomiald> p_vec {yyyy};
  BuildSource(PiecewisePolynomial<double>(p_vec, {0.0, 3}));

  ASSERT_EQ(0, context_->num_input_ports());

  const double kTestTime = 1.75;
  context_->SetTime(kTestTime);

  const auto& output = source_->get_output_port().Eval(*context_);

  int len = kppTraj_->rows();
  EXPECT_EQ(kppTraj_->value(kTestTime), output.segment(0, len));

  for (size_t i = 1; i <= kDerivativeOrder; ++i) {
    EXPECT_TRUE(
        CompareMatrices(kppTraj_->derivative(i).value(kTestTime),
                        output.segment(len * i, len), 1e-10,
                        MatrixCompareType::absolute));
  }

  // Test first derivative (which is in second segment):
  // f`(yyyy) = f`(y^4) = 4 * y^3.  y = kTestTime.
  EXPECT_NEAR(output.segment(len, len)(0), 21.4375, 1e-6);

  Eigen::MatrixXd samples(1, 2);
  samples << 0, 1;
  PiecewisePolynomial<double> pp = PiecewisePolynomial<double>::FirstOrderHold(
      Eigen::Vector2d(0, 2), samples);
  source_->UpdateTrajectory(pp);
  context_->SetTime(kTestTime);  // Set the time again to invalidate the cache.

  const auto& output2 = source_->get_output_port().Eval(*context_);
  EXPECT_EQ(pp.value(kTestTime), output2.segment(0, len));
  for (size_t i = 1; i <= kDerivativeOrder; ++i) {
    EXPECT_TRUE(
        CompareMatrices(pp.derivative(i).value(kTestTime),
                        output2.segment(len * i, len), 1e-10,
                        MatrixCompareType::absolute));
  }
}

// Tests that ConstantVectorSource allocates no state variables in the context_.
TEST_F(TrajectorySourceTest, ConstantVectorSourceIsStateless) {
  BuildSource(PiecewisePolynomial<double>(MatrixXd::Constant(2, 1, 1.5)));
  EXPECT_EQ(0, context_->num_continuous_states());
}

}  // namespace
}  // namespace systems
}  // namespace drake
