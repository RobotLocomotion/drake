#include "drake/system1/feedback_system.h"

#include "drake/system1/LinearSystem.h"
#include "drake/systems/test/system_test_util.h"
#include "gtest/gtest.h"

using Eigen::Dynamic;

namespace drake {
namespace {

template <int NumStates1, int NumInputs1, int NumOutputs1, int NumStates2>
std::shared_ptr<
    FeedbackSystem<AffineSystem<EigenVector<NumStates1>::template type,
                                EigenVector<NumInputs1>::template type,
                                EigenVector<NumOutputs1>::template type>,
                   AffineSystem<EigenVector<NumStates2>::template type,
                                EigenVector<NumOutputs1>::template type,
                                EigenVector<NumInputs1>::template type>>>
MakeFeedbackLoopOfAffineSystems(size_t num_states_1, size_t num_inputs_1,
                                size_t num_outputs_1, size_t num_states_2,
                                bool feedthrough_1, bool feedthrough_2) {
  auto sys1_ptr = system_test::CreateRandomAffineSystem<NumStates1, NumInputs1,
                                                        NumOutputs1>(
      num_states_1, num_inputs_1, num_outputs_1, feedthrough_1);
  auto sys2_ptr = system_test::CreateRandomAffineSystem<NumStates2, NumOutputs1,
                                                        NumInputs1>(
      num_states_2, num_outputs_1, num_inputs_1, feedthrough_2);
  return feedback(sys1_ptr, sys2_ptr);
}

// Tests that, if the number of inputs and outputs for each system in the
// feedback loop is fixed at compile time, the number of states, inputs, and
// outputs for the combined system matches up.
GTEST_TEST(FeedbackSystemTest, ConstantSizes) {
  auto combined =
      MakeFeedbackLoopOfAffineSystems<4, 2, 3, 5>(4, 2, 3, 5, true, false);
  EXPECT_EQ(4u + 5u, getNumStates(*combined));
  EXPECT_EQ(4u + 5u, createStateVector<double>(*combined).size());
  EXPECT_EQ(2u, getNumInputs(*combined));
  EXPECT_EQ(3u, getNumOutputs(*combined));
}

// Tests that, if the number of inputs and outputs for each system in the
// feedback loop is determined at construction, the number of states, inputs,
// and outputs for the combined system matches up.
GTEST_TEST(FeedbackSystemTest, DynamicSizes) {
  auto combined =
      MakeFeedbackLoopOfAffineSystems<Dynamic, Dynamic, Dynamic, Dynamic>(
          4, 2, 3, 5, false, true);
  EXPECT_EQ(4u + 5u, getNumStates(*combined));
  EXPECT_EQ(4u + 5u, createStateVector<double>(*combined).size());
  EXPECT_EQ(2u, getNumInputs(*combined));
  EXPECT_EQ(3u, getNumOutputs(*combined));
}

// Tests that algebraic loops are rejected.
GTEST_TEST(FeedbackSystemTest, AlgebraicLoop) {
  EXPECT_THROW(
      (MakeFeedbackLoopOfAffineSystems<1, 1, 1, 1>(1, 1, 1, 1, true, true)),
      std::exception);
}

}  // namespace
}  // namespace drake
