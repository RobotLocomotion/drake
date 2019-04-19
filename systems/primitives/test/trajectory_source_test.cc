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

  void Reset(PiecewisePolynomial<double> pp) {
    kppTraj_ = make_unique<PiecewisePolynomial<double>>(pp);
    source_ = make_unique<TrajectorySource<double>>(*kppTraj_, kDerivativeOrder,
                                                    true);
    context_ = source_->CreateDefaultContext();
    input_ = make_unique<BasicVector<double>>(3 /* length */);
  }

  std::unique_ptr<PiecewisePolynomial<double>> kppTraj_;
  std::unique_ptr<System<double>> source_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<BasicVector<double>> input_;
};

TEST_F(TrajectorySourceTest, OutputTest) {
  const Polynomiald y = Polynomiald("y");
  const Polynomiald yyyy = (y * y * y * y);
  const std::vector<Polynomiald> p_vec {yyyy};
  Reset(PiecewisePolynomial<double>(p_vec, {0.0, 3}));

  ASSERT_EQ(0, context_->num_input_ports());

  const double kTestTime = 1.75;
  context_->SetTime(kTestTime);

  const auto& output = source_->get_output_port(0).Eval(*context_);

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
}

// Tests that ConstantVectorSource allocates no state variables in the context_.
TEST_F(TrajectorySourceTest, ConstantVectorSourceIsStateless) {
  Reset(PiecewisePolynomial<double>(MatrixXd::Constant(2, 1, 1.5)));
  EXPECT_EQ(0, context_->num_continuous_states());
}

}  // namespace
}  // namespace systems
}  // namespace drake
