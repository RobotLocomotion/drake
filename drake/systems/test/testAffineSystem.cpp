#include "drake/util/testUtil.h"
#include "drake/systems/LinearSystem.h"
#include <iostream>

using namespace std;
using namespace Drake;
using namespace Eigen;

template <int StatesAtCompileTime, int InputsAtCompileTime, int OutputsAtCompileTime>
shared_ptr<AffineSystem<
      EigenVector<StatesAtCompileTime>::template type,
      EigenVector<InputsAtCompileTime>::template type,
      EigenVector<OutputsAtCompileTime>::template type >>
createRandomAffineSystem(size_t num_states, size_t num_inputs, size_t num_outputs) {
  using ReturnType = AffineSystem<
      EigenVector<StatesAtCompileTime>::template type,
      EigenVector<InputsAtCompileTime>::template type,
      EigenVector<OutputsAtCompileTime>::template type >;

  auto A = Matrix<double, StatesAtCompileTime, StatesAtCompileTime>::Random(num_states, num_states).eval();
  auto B = Matrix<double, StatesAtCompileTime, InputsAtCompileTime>::Random(num_states, num_inputs).eval();
  auto C = Matrix<double, OutputsAtCompileTime, StatesAtCompileTime>::Random(num_outputs, num_states).eval();
  auto D = Matrix<double, OutputsAtCompileTime, InputsAtCompileTime>::Random(num_outputs, num_inputs).eval();
  auto xdot0 = Matrix<double, StatesAtCompileTime, 1>::Random(num_states, 1).eval();
  auto y0 = Matrix<double, OutputsAtCompileTime, 1>::Random(num_outputs, 1).eval();
  return allocate_shared<ReturnType>(aligned_allocator<ReturnType>(), A, B, xdot0, C, D, y0);
};


template <int StatesAtCompileTime, int InputsAtCompileTime, int OutputsAtCompileTime>
void testSizes(size_t num_states, size_t num_inputs, size_t num_outputs) {
  auto sys_ptr = createRandomAffineSystem<StatesAtCompileTime, InputsAtCompileTime, OutputsAtCompileTime>(num_states, num_inputs, num_outputs);
  const auto& sys = *sys_ptr;
  using SysType = typename remove_reference<decltype(sys)>::type;

  valuecheck(num_states, sys.getNumStates(), "Wrong number of states.");
  valuecheck(num_inputs, sys.getNumInputs(), "Wrong number of inputs.");
  valuecheck(num_outputs, sys.getNumOutputs(), "Wrong number of outputs.");

  valuecheck(num_states, getNumStates(sys));
  valuecheck(num_inputs, getNumInputs(sys));
  valuecheck(num_outputs, getNumOutputs(sys));

  auto x = createStateVector<double>(sys);
  valuecheck(num_states, static_cast<size_t>(x.size()), "State vector size wrong");
};

template <int NumStates1, int NumInputs1, int NumOutputs1, int NumStates2, int NumOutputs2>
void testCascade(size_t num_states_1, size_t num_inputs_1, size_t num_outputs_1, size_t num_states_2, size_t num_outputs_2) {
  auto sys1_ptr = createRandomAffineSystem<NumStates1, NumInputs1, NumOutputs1>(num_states_1, num_inputs_1, num_outputs_1);
  auto sys2_ptr = createRandomAffineSystem<NumStates2, NumOutputs1, NumOutputs2>(num_states_2, num_outputs_1, num_outputs_2);

  auto combined = cascade(sys1_ptr, sys2_ptr);

  valuecheck(num_states_1 + num_states_2, getNumStates(*combined), "Wrong number of states");
  valuecheck(num_inputs_1, getNumInputs(*combined), "Wrong number of inputs");
  valuecheck(num_outputs_2, getNumOutputs(*combined), "Wrong number of outputs");

  auto x = createStateVector<double>(*combined);
  valuecheck(getNumStates(*combined), static_cast<size_t>(x.size()), "State vector size wrong");
}

template <int NumStates1, int NumInputs1, int NumOutputs1, int NumStates2>
void testFeedback(size_t num_states_1, size_t num_inputs_1, size_t num_outputs_1, size_t num_states_2) {
  auto sys1_ptr = createRandomAffineSystem<NumStates1, NumInputs1, NumOutputs1>(num_states_1, num_inputs_1, num_outputs_1);
  auto sys2_ptr = createRandomAffineSystem<NumStates2, NumOutputs1, NumInputs1>(num_states_2, num_outputs_1, num_inputs_1);

  auto combined = feedback(sys1_ptr, sys2_ptr);

  valuecheck(num_states_1 + num_states_2, getNumStates(*combined), "Wrong number of states");
  valuecheck(num_inputs_1, getNumInputs(*combined), "Wrong number of inputs");
  valuecheck(num_outputs_1, getNumOutputs(*combined), "Wrong number of outputs");

  auto x = createStateVector<double>(*combined);
  valuecheck(getNumStates(*combined), static_cast<size_t>(x.size()), "State vector size wrong");
}

int main(int argc, char* argv[]) {
  testSizes<Dynamic, Dynamic, Dynamic>(3, 4, 5);
  testSizes<3, 4, 5>(3, 4, 5);
  testCascade<3, 4, 5, 6, 7>(3, 4, 5, 6, 7);
  testCascade<Dynamic, Dynamic, Dynamic, Dynamic, Dynamic>(3, 4, 5, 6, 7);
  testFeedback<4, 2, 3, 5>(4, 2, 3, 5);
  testFeedback<Dynamic, Dynamic, Dynamic, Dynamic>(4, 2, 3, 5);

  return 0;
}
