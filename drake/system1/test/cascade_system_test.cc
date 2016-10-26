#include "drake/system1/cascade_system.h"

#include "drake/system1/LinearSystem.h"
#include "drake/systems/test/system_test_util.h"
#include "gtest/gtest.h"

using Eigen::Dynamic;

namespace drake {
namespace {

template <int NumStates1, int NumInputs1, int NumOutputs1, int NumStates2,
          int NumOutputs2>
std::shared_ptr<
    CascadeSystem<AffineSystem<EigenVector<NumStates1>::template type,
                               EigenVector<NumInputs1>::template type,
                               EigenVector<NumOutputs1>::template type>,
                  AffineSystem<EigenVector<NumStates2>::template type,
                               EigenVector<NumOutputs1>::template type,
                               EigenVector<NumOutputs2>::template type>>>
MakeCascadeOfAffineSystems(size_t num_states_1, size_t num_inputs_1,
                           size_t num_outputs_1, size_t num_states_2,
                           size_t num_outputs_2) {
  auto sys1_ptr = system_test::CreateRandomAffineSystem<NumStates1, NumInputs1,
                                                        NumOutputs1>(
      num_states_1, num_inputs_1, num_outputs_1);
  auto sys2_ptr = system_test::CreateRandomAffineSystem<NumStates2, NumOutputs1,
                                                        NumOutputs2>(
      num_states_2, num_outputs_1, num_outputs_2);
  return cascade(sys1_ptr, sys2_ptr);
}

// Tests that, if the number of inputs and outputs for each element in the
// cascade is fixed at compile time, the number of states, inputs, and
// outputs for the combined system matches up.
GTEST_TEST(CascadeSystemTest, ConstantSizes) {
  auto combined = MakeCascadeOfAffineSystems<3, 4, 5, 6, 7>(3, 4, 5, 6, 7);
  EXPECT_EQ(3u + 6u, getNumStates(*combined));
  EXPECT_EQ(3u + 6u, createStateVector<double>(*combined).size());
  EXPECT_EQ(4u, getNumInputs(*combined));
  EXPECT_EQ(7u, getNumOutputs(*combined));
}

// Tests that, if the number of inputs and outputs for each element in the
// cascade is determined at construction, the number of states, inputs, and
// outputs for the combined system matches up.
GTEST_TEST(CascadeSystemTest, DynamicSizes) {
  auto combined =
      MakeCascadeOfAffineSystems<Dynamic, Dynamic, Dynamic, Dynamic, Dynamic>(
          3, 4, 5, 6, 7);
  EXPECT_EQ(3u + 6u, getNumStates(*combined));
  EXPECT_EQ(3u + 6u, createStateVector<double>(*combined).size());
  EXPECT_EQ(4u, getNumInputs(*combined));
  EXPECT_EQ(7u, getNumOutputs(*combined));
}

}  // namespace
}  // namespace drake
