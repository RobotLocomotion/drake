#include "drake/systems/cascade_system.h"

#include "drake/systems/LinearSystem.h"
#include "drake/systems/test/system_test_util.h"
#include "gtest/gtest.h"

using Eigen::Dynamic;

namespace Drake {
namespace {

/*
template <int NumStates1, int NumInputs1, int NumOutputs1, int NumStates2, int
NumOutputs2>
void testCascade(size_t num_states_1, size_t num_inputs_1, size_t num_outputs_1,
size_t num_states_2, size_t num_outputs_2) {
  valuecheck(num_states_1 + num_states_2, getNumStates(*combined), "Wrong number
of states");
  valuecheck(num_inputs_1, getNumInputs(*combined), "Wrong number of inputs");
  valuecheck(num_outputs_2, getNumOutputs(*combined), "Wrong number of
outputs");

  auto x = createStateVector<double>(*combined);
  valuecheck(getNumStates(*combined), static_cast<size_t>(x.size()), "State
vector size wrong");
}
*/

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

TEST(CascadeSystemTest, ConstantSizes) {
  auto combined = MakeCascadeOfAffineSystems<3, 4, 5, 6, 7>(3, 4, 5, 6, 7);
  EXPECT_EQ(3 + 6, getNumStates(*combined));
  EXPECT_EQ(3 + 6, createStateVector<double>(*combined).size());
  EXPECT_EQ(4, getNumInputs(*combined));
  EXPECT_EQ(7, getNumOutputs(*combined));
}

TEST(CascadeSystemTest, DynamicSizes) {
  auto combined =
      MakeCascadeOfAffineSystems<Dynamic, Dynamic, Dynamic, Dynamic, Dynamic>(
          3, 4, 5, 6, 7);
  EXPECT_EQ(3 + 6, getNumStates(*combined));
  EXPECT_EQ(3 + 6, createStateVector<double>(*combined).size());
  EXPECT_EQ(4, getNumInputs(*combined));
  EXPECT_EQ(7, getNumOutputs(*combined));
}

}  // namespace
}  // namespace Drake
