#include "drake/multibody/plant/rigid_transform_selector.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Vector3d;
using math::RigidTransform;
using ListType = std::vector<RigidTransform<double>>;

GTEST_TEST(RigidTransformSelectorTest, BasicTest) {
  const int kIndex = 1;
  RigidTransformSelector<double> dut(kIndex);
  EXPECT_EQ(dut.num_input_ports(), 1);
  EXPECT_EQ(dut.num_output_ports(), 1);
  EXPECT_TRUE(dut.HasDirectFeedthrough(0, 0));

  ListType vector(2);
  vector[kIndex] = RigidTransform<double>(Vector3d(0.1, 0.2, 0.3));

  auto context = dut.CreateDefaultContext();
  dut.get_input_port().FixValue(context.get(), vector);
  auto X =
      dut.get_output_port().template Eval<RigidTransform<double>>(*context);
  EXPECT_TRUE(X.IsExactlyEqualTo(vector[kIndex]));

  ListType empty_vector;
  dut.get_input_port().FixValue(context.get(), empty_vector);
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.get_output_port().template Eval<RigidTransform<double>>(*context),
      ".*cannot select index.*");

  EXPECT_TRUE(systems::is_autodiffxd_convertible(dut));
  EXPECT_TRUE(systems::is_symbolic_convertible(dut));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
