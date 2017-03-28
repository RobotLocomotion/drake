#include "drake/examples/kuka_iiwa_arm/iiwa_plan_source.h"

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

static const int kNumJoints = 7;
const char* const kIiwaUrdf =
    "/manipulation/models/iiwa_description/urdf/"
        "iiwa14_polytope_collision.urdf";

GTEST_TEST(IiwaPlanSourceTest, InstanceTest) {
  // Test that the constructor works and that the expected ports are
  // present.
  IiwaPlanSource dut(GetDrakePath() + kIiwaUrdf);
  EXPECT_EQ(dut.get_plan_input_port().get_data_type(),
            systems::kAbstractValued);
  EXPECT_EQ(dut.get_status_input_port().get_data_type(),
            systems::kAbstractValued);
  EXPECT_EQ(dut.get_output_port(0).get_data_type(),
            systems::kVectorValued);
  EXPECT_EQ(dut.get_output_port(0).size(), kNumJoints * 2);
}

// TODO(sam.creasey) Add a more meaningful test.

}  // namespace

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
