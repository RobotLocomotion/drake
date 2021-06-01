#include "drake/systems/primitives/linear_transform_density.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace systems {
template <typename T>
void TestConstructor() {
  LinearTransformDensity<T> dut(RandomDistribution::kUniform,
                                2 /* input_size */, 3 /* output_size */);
  EXPECT_EQ(dut.num_input_ports(), 3);
  EXPECT_EQ(dut.num_output_ports(), 1);
  EXPECT_EQ(dut.get_distribution(), RandomDistribution::kUniform);

  EXPECT_EQ(dut.get_input_port_w_in().size(), 2);
  EXPECT_EQ(dut.get_input_port_A().size(), 6);
  EXPECT_EQ(dut.get_input_port_b().size(), 3);
  EXPECT_EQ(dut.get_output_port(0).size(), 3);
}

GTEST_TEST(LinearTransformDensityTest, Constructor) {
  TestConstructor<double>();
  TestConstructor<AutoDiffXd>();
}

GTEST_TEST(LinearTransformDensityTest, ToAutoDiff) {
  auto dut = std::make_unique<LinearTransformDensity<double>>(
      RandomDistribution::kUniform, 2, 3);
  EXPECT_TRUE(is_autodiffxd_convertible(*dut));
}

template <typename T>
void TestCalcOutput() {
  LinearTransformDensity<T> dut(RandomDistribution::kUniform, 2, 3);
  auto context = dut.CreateDefaultContext();

  Eigen::Matrix<T, 3, 2> A;
  A << T(1.), T(2.), T(3.), T(4.), T(5.), T(6.);
  dut.get_input_port_A().FixValue(context.get(),
                                  Eigen::Map<Vector6<T>>(A.data()));
  Vector3<T> b(-1, -2, -3);
  dut.get_input_port_b().FixValue(context.get(), b);

  Vector2<T> w_in(2, 3);
  dut.get_input_port_w_in().FixValue(context.get(), w_in);

  const auto w_out = dut.get_output_port().Eval(*context);
  const Vector3<T> w_out_expected = A * w_in + b;
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(ExtractDoubleOrThrow(w_out(i)),
              ExtractDoubleOrThrow(w_out_expected(i)));
  }
}

GTEST_TEST(LinearTransformDensityTest, CalcOutput) {
  TestCalcOutput<double>();
  TestCalcOutput<AutoDiffXd>();
}
}  // namespace systems
}  // namespace drake
