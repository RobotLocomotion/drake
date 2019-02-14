#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

// Builds a RBT with two iiwas, but only select the second iiwa's generalized
// force when converting to external joint torque.
GTEST_TEST(IiwaLcmTest, IiwaContactResultsToExternalTorque) {
  const std::string kIiwaUrdf =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";
  auto tree_builder =
      std::make_unique<manipulation::util::WorldSimTreeBuilder<double>>();
  tree_builder->StoreDrakeModel("iiwa", kIiwaUrdf);

  tree_builder->AddFixedModelInstance("iiwa", Vector3<double>::Zero());
  int id1 =
      tree_builder->AddFixedModelInstance("iiwa", Vector3<double>(0, 0, 1));

  auto tree = tree_builder->Build();

  // Only interested in the second iiwa's external torque output.
  IiwaContactResultsToExternalTorque dut(*tree, {id1});
  std::unique_ptr<systems::Context<double>>
      context = dut.CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      dut.AllocateOutput();

  VectorX<double> expected(tree->get_num_velocities());
  for (int i = 0; i < expected.size(); i++)
    expected[i] = 42 + i;
  systems::ContactResults<double> contact_results;
  contact_results.set_generalized_contact_force(expected);

  context->FixInputPort(0,
      AbstractValue::Make<systems::ContactResults<double>>(
          contact_results));

  // Check output.
  dut.CalcOutput(*context, output.get());
  const auto ext_torque = output->get_vector_data(0)->get_value();
  EXPECT_EQ(ext_torque.size(), 7);
  for (int i = 0; i < 7; i++) {
    EXPECT_EQ(ext_torque[i], expected[7 + i]);
  }
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
