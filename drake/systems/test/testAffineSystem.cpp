#include <iostream>

#include "drake/util/testUtil.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/feedback_system.h"
#include "drake/systems/test/system_test_util.h"

using namespace std;
using namespace Drake;
using namespace Eigen;

template <int StatesAtCompileTime, int InputsAtCompileTime,
          int OutputsAtCompileTime>
shared_ptr<AffineSystem<EigenVector<StatesAtCompileTime>::template type,
                        EigenVector<InputsAtCompileTime>::template type,
                        EigenVector<OutputsAtCompileTime>::template type>>
createRandomAffineSystem(size_t num_states, size_t num_inputs,
                         size_t num_outputs) {
  using ReturnType =
      AffineSystem<EigenVector<StatesAtCompileTime>::template type,
                   EigenVector<InputsAtCompileTime>::template type,
                   EigenVector<OutputsAtCompileTime>::template type>;

  auto A = Matrix<double, StatesAtCompileTime, StatesAtCompileTime>::Random(
               num_states, num_states).eval();
  auto B = Matrix<double, StatesAtCompileTime, InputsAtCompileTime>::Random(
               num_states, num_inputs).eval();
  auto C = Matrix<double, OutputsAtCompileTime, StatesAtCompileTime>::Random(
               num_outputs, num_states).eval();
  auto D = Matrix<double, OutputsAtCompileTime, InputsAtCompileTime>::Random(
               num_outputs, num_inputs).eval();
  auto xdot0 =
      Matrix<double, StatesAtCompileTime, 1>::Random(num_states, 1).eval();
  auto y0 =
      Matrix<double, OutputsAtCompileTime, 1>::Random(num_outputs, 1).eval();
  return allocate_shared<ReturnType>(aligned_allocator<ReturnType>(), A, B,
                                     xdot0, C, D, y0);
}

template <int StatesAtCompileTime, int InputsAtCompileTime,
          int OutputsAtCompileTime>
void testSizes(size_t num_states, size_t num_inputs, size_t num_outputs) {
  auto sys_ptr =
      createRandomAffineSystem<StatesAtCompileTime, InputsAtCompileTime,
                               OutputsAtCompileTime>(num_states, num_inputs,
                                                     num_outputs);
  const auto& sys = *sys_ptr;

  valuecheck(num_states, sys.getNumStates(), "Wrong number of states.");
  valuecheck(num_inputs, sys.getNumInputs(), "Wrong number of inputs.");
  valuecheck(num_outputs, sys.getNumOutputs(), "Wrong number of outputs.");

  valuecheck(num_states, getNumStates(sys));
  valuecheck(num_inputs, getNumInputs(sys));
  valuecheck(num_outputs, getNumOutputs(sys));

  auto x = createStateVector<double>(sys);
  valuecheck(num_states, static_cast<size_t>(x.size()),
             "State vector size wrong");
}

int main(int argc, char* argv[]) {
  testSizes<Dynamic, Dynamic, Dynamic>(3, 4, 5);
  testSizes<3, 4, 5>(3, 4, 5);
  return 0;
}
