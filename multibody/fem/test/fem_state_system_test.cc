#include "drake/multibody/fem/fem_state_system.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

constexpr int kNumDofs = 3;

VectorXd q() {
  Vector<double, kNumDofs> q;
  q << 0.1, 0.2, 0.3;
  return q;
}

VectorXd v() {
  Vector<double, kNumDofs> v;
  v << 1.1, 1.2, 2.3;
  return v;
}

VectorXd a() {
  Vector<double, kNumDofs> a;
  a << 2.1, 2.2, 3.3;
  return a;
}

GTEST_TEST(FemStateSystemTest, Constructor) {
  FemStateSystem<double> fem_state_system(q(), v(), a());
  EXPECT_EQ(fem_state_system.num_dofs(), kNumDofs);
  /* An exception is thrown if the number of dofs isn't consistent across
   * states. */
  EXPECT_THROW(FemStateSystem<double>(Vector4d::Zero(), v(), a()),
               std::exception);
  EXPECT_THROW(FemStateSystem<double>(q(), Vector4d::Zero(), v()),
               std::exception);
  EXPECT_THROW(FemStateSystem<double>(q(), v(), Vector4d::Zero()),
               std::exception);
  /* An exception is thrown if the number of dofs isn't a multiple of 3. */
  EXPECT_THROW(FemStateSystem<double>(Vector4d::Zero(), Vector4d::Zero(),
                                      Vector4d::Zero()),
               std::exception);
}

/* Verifies that discrete states can be retrieved via their indexes. */
GTEST_TEST(FemStateSystemTest, StateIndexes) {
  FemStateSystem<double> fem_state_system(q(), v(), a());
  auto context = fem_state_system.CreateDefaultContext();
  EXPECT_EQ(context->get_discrete_state(fem_state_system.fem_position_index())
                .value(),
            q());
  EXPECT_EQ(context->get_discrete_state(fem_state_system.fem_velocity_index())
                .value(),
            v());
  EXPECT_EQ(
      context->get_discrete_state(fem_state_system.fem_acceleration_index())
          .value(),
      a());
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
