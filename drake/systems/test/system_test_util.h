#ifndef DRAKE_SYSTEMS_TEST_SYSTEM_TEST_UTIL_H_
#define DRAKE_SYSTEMS_TEST_SYSTEM_TEST_UTIL_H_

#include "drake/systems/LinearSystem.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/feedback_system.h"
#include "drake/systems/test/system_test_util.h"

namespace Drake {
namespace system_test {

template <int StatesAtCompileTime, int InputsAtCompileTime,
          int OutputsAtCompileTime>
std::shared_ptr<AffineSystem<EigenVector<StatesAtCompileTime>::template type,
                             EigenVector<InputsAtCompileTime>::template type,
                             EigenVector<OutputsAtCompileTime>::template type>>
CreateRandomAffineSystem(size_t num_states, size_t num_inputs,
                         size_t num_outputs) {
  using ReturnType =
      AffineSystem<EigenVector<StatesAtCompileTime>::template type,
                   EigenVector<InputsAtCompileTime>::template type,
                   EigenVector<OutputsAtCompileTime>::template type>;

  auto A =
      Eigen::Matrix<double, StatesAtCompileTime, StatesAtCompileTime>::Random(
          num_states, num_states)
          .eval();
  auto B =
      Eigen::Matrix<double, StatesAtCompileTime, InputsAtCompileTime>::Random(
          num_states, num_inputs)
          .eval();
  auto C =
      Eigen::Matrix<double, OutputsAtCompileTime, StatesAtCompileTime>::Random(
          num_outputs, num_states)
          .eval();
  auto D =
      Eigen::Matrix<double, OutputsAtCompileTime, InputsAtCompileTime>::Random(
          num_outputs, num_inputs)
          .eval();
  auto xdot0 =
      Eigen::Matrix<double, StatesAtCompileTime, 1>::Random(num_states, 1)
          .eval();
  auto y0 =
      Eigen::Matrix<double, OutputsAtCompileTime, 1>::Random(num_outputs, 1)
          .eval();
  return std::allocate_shared<ReturnType>(
      Eigen::aligned_allocator<ReturnType>(), A, B, xdot0, C, D, y0);
}

}  // namespace system_test
}  // namespace Drake

#endif  // DRAKE_SYSTEMS_TEST_SYSTEM_TEST_UTIL_H_
