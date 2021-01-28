#include "drake/multibody/fixed_fem/dev/zeroth_order_state_updater.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/fixed_fem/dev/test/dummy_element.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
namespace test {
namespace {
class ZerothOrderStateUpdaterTest : public ::testing::Test {
 protected:
  ZerothOrderStateUpdater<FemState<DummyElement<0>>> state_updater;
};

TEST_F(ZerothOrderStateUpdaterTest, StateDerivative) {
  EXPECT_TRUE(
      CompareMatrices(state_updater.weights(), Vector3<double>(1, 0, 0), 0));
}

TEST_F(ZerothOrderStateUpdaterTest, UpdateState) {
  VectorX<double> q = Vector4<double>(1.23, 2.34, 3.45, 4.56);
  VectorX<double> dq = Vector4<double>(0.23, 0.34, 0.45, 0.56);
  FemState<DummyElement<0>> state(q);
  state_updater.UpdateState(dq, &state);
  EXPECT_TRUE(CompareMatrices(state.q(), q + dq, 0));
}
}  // namespace
}  // namespace test
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
