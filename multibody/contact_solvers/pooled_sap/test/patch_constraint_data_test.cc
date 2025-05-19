#include "drake/multibody/contact_solvers/pooled_sap/patch_constraint_data.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

GTEST_TEST(ConstraintDataTest, Construction) {
  // Pool for three constraints.
  std::vector<int> patch_size = {3, 2, 4};
  std::vector<int> num_velocities = {12, 7, 9};
  PatchConstraintsDataPool<double> pool(patch_size, num_velocities);

  EXPECT_EQ(pool.num_patches(), 3);
  EXPECT_EQ(pool.num_pairs(), 9);
  for (int i = 0; i < pool.num_pairs(); ++i) {
    EXPECT_EQ(pool.vc(i).rows(), 3);
    EXPECT_EQ(pool.vc(i).cols(), 1);
    EXPECT_EQ(pool.gamma(i).rows(), 3);
    EXPECT_EQ(pool.gamma(i).cols(), 1);
  }
}

GTEST_TEST(ConstraintDataTest, MutateElements) {
  std::vector<int> patch_size = {3, 2, 4};
  std::vector<int> num_velocities = {12, 7, 9};
  PatchConstraintsDataPool<double> pool(patch_size, num_velocities);

  ASSERT_EQ(pool.num_patches(), 3);
  for (int i = 0; i < pool.num_pairs(); ++i) {
    const Vector3d vc = Vector3d::Constant(2.1 * i + 0.1);
    pool.vc(i) = vc;
    EXPECT_EQ(pool.vc(i), vc);

    const Vector3d gamma = -3.4 * vc;
    pool.gamma(i) = gamma;
    EXPECT_EQ(pool.gamma(i), gamma);

    // const int nv = num_velocities[i];
    // const MatrixXd H = MatrixXd::Constant(nv, nv, 3.5 * i - 0.15);
    // pool.H(i) = H;
    // EXPECT_EQ(pool.H(i), H);
  }
}

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
