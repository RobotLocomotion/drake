#include "drake/system1/LinearSystem.h"

#include "gtest/gtest.h"

#include "drake/system1/cascade_system.h"
#include "drake/system1/feedback_system.h"
#include "drake/systems/test/system_test_util.h"

// NOLINTNEXTLINE(build/namespaces) This code will be deleted soon.
using namespace std;
// NOLINTNEXTLINE(build/namespaces) This code will be deleted soon.
using namespace Eigen;

namespace drake {
namespace {

// TODO()jwnimmer-tri) Unit tests should not use unseeded randomness.
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

  EXPECT_EQ(num_states, sys.getNumStates());
  EXPECT_EQ(num_inputs, sys.getNumInputs());
  EXPECT_EQ(num_outputs, sys.getNumOutputs());

  EXPECT_EQ(num_states, getNumStates(sys));
  EXPECT_EQ(num_inputs, getNumInputs(sys));
  EXPECT_EQ(num_outputs, getNumOutputs(sys));

  auto x = createStateVector<double>(sys);
  EXPECT_EQ(num_states, static_cast<size_t>(x.size()));
}

GTEST_TEST(AffineSystemTest, RandomDynamicSize) {
  testSizes<Dynamic, Dynamic, Dynamic>(3, 4, 5);
}

GTEST_TEST(AffineSystemTest, RandomFixedSize) {
  testSizes<3, 4, 5>(3, 4, 5);
}

}  // namespace
}  // namespace drake
