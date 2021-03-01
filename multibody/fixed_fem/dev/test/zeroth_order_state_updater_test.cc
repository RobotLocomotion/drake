#include "drake/multibody/fixed_fem/dev/zeroth_order_state_updater.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
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

TEST_F(ZerothOrderStateUpdaterTest, Weights) {
  EXPECT_TRUE(
      CompareMatrices(state_updater.weights(), Vector3<double>(1, 0, 0), 0));
}

TEST_F(ZerothOrderStateUpdaterTest, UpdateStateFromChangeInUnknowns) {
  const VectorX<double> q = Vector4<double>(1.23, 2.34, 3.45, 4.56);
  const VectorX<double> dq = Vector4<double>(0.23, 0.34, 0.45, 0.56);
  FemState<DummyElement<0>> state(q);
  state_updater.UpdateStateFromChangeInUnknowns(dq, &state);
  EXPECT_TRUE(CompareMatrices(state.q(), q + dq, 0));
}

TEST_F(ZerothOrderStateUpdaterTest, AdvanceOneTimeStep) {
  const VectorX<double> q = Vector4<double>(1.23, 2.34, 3.45, 4.56);
  FemState<DummyElement<0>> prev_state(q);
  FemState<DummyElement<0>> new_state(prev_state);
  DRAKE_EXPECT_THROWS_MESSAGE(
      state_updater.AdvanceOneTimeStep(prev_state, q, &new_state),
      std::exception, "There is no notion of time in a zeroth order ODE.");
}
}  // namespace
}  // namespace test
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
