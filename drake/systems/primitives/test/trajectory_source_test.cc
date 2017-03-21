#include "drake/systems/primitives/trajectory_source.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/input_port_value.h"

using Eigen::Matrix;
using Eigen::MatrixXd;
using std::make_unique;

namespace drake {
namespace systems {
namespace {

class TrajectorySourceTest : public ::testing::Test {
 protected:
  const size_t kDerivativeOrder = 5;

  void Reset(PiecewisePolynomial<double> pp) {
    kppTraj_ = make_unique<PiecewisePolynomialTrajectory>(pp);
    source_ = make_unique<TrajectorySource<double>>(*kppTraj_, kDerivativeOrder,
                                                    true);
    context_ = source_->CreateDefaultContext();
    output_ = source_->AllocateOutput(*context_);
    input_ = make_unique<BasicVector<double>>(3 /* length */);
  }

  std::unique_ptr<PiecewisePolynomialTrajectory> kppTraj_;
  std::unique_ptr<System<double>> source_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<BasicVector<double>> input_;
};

TEST_F(TrajectorySourceTest, OutputTest) {
  const Polynomiald y = Polynomiald("y");
  const Polynomiald yyyy = (y * y * y * y);
  const std::vector<Polynomiald> p_vec {yyyy};
  Reset(PiecewisePolynomial<double>(p_vec, {0.0, 3}));

  ASSERT_EQ(0, context_->get_num_input_ports());
  ASSERT_EQ(1, output_->get_num_ports());

  const double kTestTime = 1.75;
  context_->set_time(kTestTime);

  source_->CalcOutput(*context_, output_.get());

  const BasicVector<double>* output_vector = output_->get_vector_data(0);
  ASSERT_NE(nullptr, output_vector);

  int len = kppTraj_->rows();
  EXPECT_EQ(kppTraj_->value(kTestTime),
            output_vector->get_value().segment(0, len));

  for (size_t i = 1; i <= kDerivativeOrder; ++i) {
    EXPECT_TRUE(
        CompareMatrices(kppTraj_->derivative(i)->value(kTestTime),
                        output_vector->get_value().segment(len * i, len), 1e-10,
                        MatrixCompareType::absolute));
  }

  // Test first derivative (which is in second segment):
  // f`(yyyy) = f`(y^4) = 4 * y^3.  y = kTestTime.
  EXPECT_NEAR(output_vector->get_value().segment(len, len)(0), 21.4375, 1e-6);
}

// Tests that ConstantVectorSource allocates no state variables in the context_.
TEST_F(TrajectorySourceTest, ConstantVectorSourceIsStateless) {
  Reset(PiecewisePolynomial<double>(MatrixXd::Constant(2, 1, 1.5)));
  EXPECT_EQ(0, context_->get_continuous_state()->size());
}

}  // namespace
}  // namespace systems
}  // namespace drake
