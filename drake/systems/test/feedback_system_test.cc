#include "drake/systems/feedback_system.h"

#include "drake/systems/LinearSystem.h"
#include "drake/systems/test/system_test_util.h"
#include "gtest/gtest.h"

using Eigen::Dynamic;

namespace Drake {
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
                                size_t num_outputs_1, size_t num_states_2) {
  auto sys1_ptr = system_test::CreateRandomAffineSystem<NumStates1, NumInputs1,
                                                        NumOutputs1>(
      num_states_1, num_inputs_1, num_outputs_1);
  auto sys2_ptr = system_test::CreateRandomAffineSystem<NumStates2, NumOutputs1,
                                                        NumInputs1>(
      num_states_2, num_outputs_1, num_inputs_1);
  return feedback(sys1_ptr, sys2_ptr);
}

// Tests that, if the number of inputs and outputs for each system in the
// feedback loop is fixed at compile time, the number of states, inputs, and
// outputs for the combined system matches up.
TEST(FeedbackSystemTest, ConstantSizes) {
  auto combined = MakeFeedbackLoopOfAffineSystems<4, 2, 3, 5>(4, 2, 3, 5);
  EXPECT_EQ(4 + 5, getNumStates(*combined));
  EXPECT_EQ(4 + 5, createStateVector<double>(*combined).size());
  EXPECT_EQ(2, getNumInputs(*combined));
  EXPECT_EQ(3, getNumOutputs(*combined));
}

// Tests that, if the number of inputs and outputs for each system in the
// feedback loop is determined at construction, the number of states, inputs,
// and outputs for the combined system matches up.
TEST(FeedbackSystemTest, DynamicSizes) {
  auto combined =
      MakeFeedbackLoopOfAffineSystems<Dynamic, Dynamic, Dynamic, Dynamic>(4, 2,
                                                                          3, 5);
  EXPECT_EQ(4 + 5, getNumStates(*combined));
  EXPECT_EQ(4 + 5, createStateVector<double>(*combined).size());
  EXPECT_EQ(2, getNumInputs(*combined));
  EXPECT_EQ(3, getNumOutputs(*combined));
}

}  // namespace
}  // namespace Drake
