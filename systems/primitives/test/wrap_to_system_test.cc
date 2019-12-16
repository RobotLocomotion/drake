#include "drake/systems/primitives/wrap_to_system.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace {

using Eigen::Vector2d;

template <typename T>
void CheckOutput(const System<T>& dut,
                 const Eigen::Ref<const VectorX<T>>& input,
                 const Eigen::Ref<const VectorX<T>>& expected_output) {
  auto context = dut.CreateDefaultContext();
  dut.get_input_port(0).FixValue(context.get(), input);
  EXPECT_TRUE(CompareMatrices(dut.get_output_port(0).Eval(*context),
                              expected_output, 1e-12));
}


template <typename T>
void SecondElementOnlyTest() {
  WrapToSystem<T> dut(2);
  dut.set_interval(1., 2., 3.);

  EXPECT_EQ(dut.get_size(), 2);

  EXPECT_EQ(dut.num_input_ports(), 1);
  EXPECT_EQ(dut.num_output_ports(), 1);

  CheckOutput<T>(dut, Vector2<T>{.1, .5}, Vector2<T>{.1, 2.5});
  CheckOutput<T>(dut, Vector2<T>{.3, -.2}, Vector2<T>{.3, 2.8});
  CheckOutput<T>(dut, Vector2<T>{.4, 32.4}, Vector2<T>{.4, 2.4});
}

GTEST_TEST(WrapToSystemTest, SecondElementOnly) {
  SecondElementOnlyTest<double>();
  SecondElementOnlyTest<AutoDiffXd>();
  SecondElementOnlyTest<symbolic::Expression>();
}

}  // namespace
}  // namespace systems
}  // namespace drake
