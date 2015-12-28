#include <testUtil.h>
#include "LinearSystem.h"
#include <iostream>

using namespace std;
using namespace Drake;
using namespace Eigen;

template <int StatesAtCompileTime, int InputsAtCompileTime, int OutputsAtCompileTime>
AffineSystem<
      EigenVector<StatesAtCompileTime>::template type,
      EigenVector<InputsAtCompileTime>::template type,
      EigenVector<OutputsAtCompileTime>::template type >
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
  return ReturnType(A, B, xdot0, C, D, y0);
};

template <typename Scalar>
using NonEigenStateVectorType = std::vector<Scalar>;

template <int StatesAtCompileTime, int InputsAtCompileTime, int OutputsAtCompileTime>
void testSizes(size_t num_states, size_t num_inputs, size_t num_outputs) {
  auto sys1 = createRandomAffineSystem<StatesAtCompileTime, InputsAtCompileTime, OutputsAtCompileTime>(num_states, num_inputs, num_outputs);
  using Sys1Type = decltype(sys1);

  valuecheck(num_states, sys1.getNumStates(), "Wrong number of states.");
  valuecheck(num_inputs, sys1.getNumInputs(), "Wrong number of inputs.");
  valuecheck(num_outputs, sys1.getNumOutputs(), "Wrong number of outputs.");

  static_assert(Sys1Type::num_states == StatesAtCompileTime, "Wrong number of states at compile time");
  static_assert(Sys1Type::num_inputs == InputsAtCompileTime, "Wrong number of inputs at compile time");
  static_assert(Sys1Type::num_outputs == OutputsAtCompileTime, "Wrong number of outputs at compile time");

  valuecheck(num_states, getNumStates(sys1));
  valuecheck(num_inputs, getNumInputs(sys1));
  valuecheck(num_outputs, getNumOutputs(sys1));

  auto x = createStateVector<double>(sys1);
  valuecheck(num_states, static_cast<size_t>(x.rows()));
};

int main(int argc, char* argv[]) {
  testSizes<Dynamic, Dynamic, Dynamic>(3, 4, 5);
  testSizes<3, 4, 5>(3, 4, 5);

  return 0;
}