#include <iostream>

#include "drake/util/testUtil.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/feedback_system.h"
#include "drake/systems/test/system_test_util.h"

using namespace std;
using namespace Drake;
using namespace Eigen;

template <int StatesAtCompileTime, int InputsAtCompileTime, int OutputsAtCompileTime>
void testSizes(size_t num_states, size_t num_inputs, size_t num_outputs) {
  auto sys_ptr = system_test::CreateRandomAffineSystem<StatesAtCompileTime, InputsAtCompileTime, OutputsAtCompileTime>(num_states, num_inputs, num_outputs);
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

int main(int argc, char* argv[]) {
  testSizes<Dynamic, Dynamic, Dynamic>(3, 4, 5);
  testSizes<3, 4, 5>(3, 4, 5);
  return 0;
}

