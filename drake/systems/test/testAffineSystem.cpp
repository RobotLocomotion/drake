#include <testUtil.h>
#include "LinearSystem.h"
#include <iostream>
#include <typeinfo>
#include <vector>

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

template <template <typename> class StateVector>
struct is_eigen_vector : public std::false_type {};

template <int Rows>
struct is_eigen_vector<EigenVector<Rows>::template type> : public std::true_type {};

template <typename Scalar>
using NonEigenStateVectorType = std::vector<Scalar>;

int main(int argc, char* argv[]) {
  size_t num_states = 3;
  size_t num_inputs = 4;
  size_t num_outputs = 5;
  auto sys1 = createRandomAffineSystem<Dynamic, Dynamic, Dynamic>(num_states, num_inputs, num_outputs);
  using Sys1Type = decltype(sys1);

  valuecheck(num_states, sys1.getNumStates(), "Wrong number of states.");
  valuecheck(num_inputs, sys1.getNumInputs(), "Wrong number of inputs.");
  valuecheck(num_outputs, sys1.getNumOutputs(), "Wrong number of outputs.");

  static_assert(Sys1Type::num_states == Dynamic, "Wrong number of states at compile time");
  static_assert(Sys1Type::num_inputs == Dynamic, "Wrong number of inputs at compile time");
  static_assert(Sys1Type::num_outputs == Dynamic, "Wrong number of outputs at compile time");

  valuecheck(num_states, getNumStates(sys1));
  valuecheck(num_inputs, getNumInputs(sys1));
  valuecheck(num_outputs, getNumOutputs(sys1));

  cout << boolalpha << is_eigen_vector<NonEigenStateVectorType>::value << endl;
  cout << boolalpha << is_eigen_vector<EigenVector<3>::type>::value << endl;



//  auto x = createStateVector<double>(sys1);
//  valuecheck(num_states, static_cast<size_t>(x.rows()));

  return 0;
}