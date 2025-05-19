#include "drake/multibody/contact_solvers/pooled_sap/constraint_data.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::VectorXd;


namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

GTEST_TEST(ConstraintDataTest, Construction) {
  // Pool for three constraints.
  std::vector<int> num_equations = {3, 2, 4};
  std::vector<int> num_velocities = {12, 7, 9};
  ConstraintDataPool<double> pool(num_equations, num_velocities);

  EXPECT_EQ(pool.num_constraints(), 3);
  for (int i = 0; i < pool.num_constraints(); ++i) {
    EXPECT_EQ(pool.vc(i).rows(), num_equations[i]);
    EXPECT_EQ(pool.vc(i).cols(), 1);
    EXPECT_EQ(pool.gamma(i).rows(), num_equations[i]);
    EXPECT_EQ(pool.gamma(i).cols(), 1);
    EXPECT_EQ(pool.H(i).rows(), num_velocities[i]);
    EXPECT_EQ(pool.H(i).cols(), num_velocities[i]);
  }
}

GTEST_TEST(ConstraintDataTest, MutateElements) {
  std::vector<int> num_equations = {3, 2, 4};
  std::vector<int> num_velocities = {12, 7, 9};
  ConstraintDataPool<double> pool(num_equations, num_velocities);

  ASSERT_EQ(pool.num_constraints(), 3);
  for (int i = 0; i < pool.num_constraints(); ++i) {
    const MatrixXd vc = VectorXd::Constant(num_equations[i], 2.1 * i + 0.1);
    pool.vc(i) = vc;
    EXPECT_EQ(pool.vc(i), vc);

    const MatrixXd gamma = -3.4 * vc;
    pool.gamma(i) = gamma;
    EXPECT_EQ(pool.gamma(i), gamma);

    const int nv = num_velocities[i];
    const MatrixXd H = MatrixXd::Constant(nv, nv, 3.5 * i - 0.15);
    pool.H(i) = H;
    EXPECT_EQ(pool.H(i), H);
  }
}

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
