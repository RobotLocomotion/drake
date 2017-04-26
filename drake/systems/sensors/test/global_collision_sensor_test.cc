#include "drake/systems/sensors/global_collision_sensor.h"

#include <memory>
#include <vector>

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/system_output.h"

using std::make_unique;
using std::string;
using std::stringstream;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace sensors {
namespace {

GTEST_TEST(TestGlobalCollisionSensor, TopologyTest) {
  const char* const kSensorName = "my collision sensor";

  auto tree_ptr = make_unique<RigidBodyTree<double>>();

  // Defines the Device Under Test (DUT).
  GlobalCollisionSensor dut(kSensorName, std::move(tree_ptr));
  const RigidBodyTree<double>& tree = dut.get_tree();

  ASSERT_EQ(dut.get_num_input_ports(), 1);
  const auto& input_descriptor = dut.get_input_port();
  EXPECT_EQ(input_descriptor.get_data_type(), systems::kVectorValued);
  EXPECT_EQ(input_descriptor.size(),
            tree.get_num_positions() + tree.get_num_velocities());

  ASSERT_EQ(dut.get_num_output_ports(), 1);
  const auto& collision_output = dut.get_output_port();
  EXPECT_EQ(collision_output.get_data_type(), systems::kAbstractValued);
}

GTEST_TEST(TestGlobalCollisionSensor, BasicCollisionTest) {
  const char* const kSensorName = "my collision sensor";

  auto tree_ptr = make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() + "/multibody/models/box.urdf",
      drake::multibody::joints::kQuaternion, nullptr /* weld to frame */,
      tree_ptr.get());
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() + "/multibody/models/box.urdf",
      drake::multibody::joints::kQuaternion, nullptr /* weld to frame */,
      tree_ptr.get());

  ASSERT_EQ(tree_ptr->get_num_model_instances(), 2);
  ASSERT_EQ(tree_ptr->get_num_positions(), 14);
  ASSERT_EQ(tree_ptr->get_num_velocities(), 12);

  // Defines the Device Under Test (DUT).
  GlobalCollisionSensor dut(kSensorName, std::move(tree_ptr));
  const RigidBodyTree<double>& tree = dut.get_tree();
  std::unique_ptr<systems::Context<double>> context =
      dut.CreateDefaultContext();

  const int num_positions = tree.get_num_positions();
  const int num_velocities = tree.get_num_velocities();
  const int num_states = num_positions + num_velocities;;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(num_states);
  x.head(num_positions) = tree.getZeroConfiguration();

  /* The indices of the position states within `x` are as follows:

        Index   Name
        ------ --------
          0     base_x
          1     base_y
          2     base_z
          3     base_qw
          4     base_qx
          5     base_qy
          6     base_qz
          7     base_x
          8     base_y
          9     base_z
          10    base_qw
          11    base_qx
          12    base_qy
          13    base_qz

    The boxes are 0.1 m wide along their x-axes (they protrude by 0.05 m on each
    side of the axis). Thus, placing one box at x = 0 and the other at x = 0.11
    will result in a separation of 0.11 m, which should be sufficient to avoid
    collision between the two boxes.
  */
  x(0) = 0;
  x(7) = 0.11;

  // Places the boxes far enough away from each other so as to avoid contact.
  // Then verifies there are no sensed collisions.
  BasicVector<double>* input_vector =
      context->FixInputPort(dut.get_input_port().get_index(), x);
  EXPECT_EQ(input_vector->size(), num_states);

  std::unique_ptr<systems::SystemOutput<double>> output =
      dut.AllocateOutput(*context);;
  dut.CalcOutput(*context, output.get());

  const AbstractValue* output_value =
      output->get_data(dut.get_output_port().get_index());
  ASSERT_NE(output_value, nullptr);
  const ContactResults<double>& contact_results =
      output_value->GetValueOrThrow<ContactResults<double>>();

  EXPECT_EQ(contact_results.get_num_contacts(), 0);

  // Places the boxes in contact with each other and verifies that the
  // GlobalCollisionSensor senses the collision. In this case, one box is placed
  // at x = -0.04 and the other box is placed at x = 0.04, resulting in a
  // separation distance of 0.08, which results in an overlap of
  // 0.08 - 0.1 = -0.02 m.
  x(0) = -0.04;
  x(7) = 0.04;
  input_vector->set_value(x);
  dut.CalcOutput(*context, output.get());
  EXPECT_EQ(contact_results.get_num_contacts(), 1);
}

GTEST_TEST(TestGlobalCollisionSensor, AttachGlobalCollisionSensorTest) {
  auto world_tree_ptr = make_unique<RigidBodyTree<double>>();
  auto collision_tree_ptr = make_unique<RigidBodyTree<double>>();
  DiagramBuilder<double> builder;
  auto plant = builder.AddSystem<RigidBodyPlant>(std::move(world_tree_ptr));
  GlobalCollisionSensor* collision_sensor =
      GlobalCollisionSensor::AttachGlobalCollisionSensor(
          "Foo Collision Sensor",
          std::move(collision_tree_ptr),
          plant->state_output_port(),
          &builder);
  EXPECT_NE(collision_sensor, nullptr);
  std::unique_ptr<Diagram<double>> diagram = builder.Build();
  ASSERT_NE(diagram.get(), nullptr);
  std::vector<const systems::System<double>*> subsystems =
      diagram->GetSystems();
  ASSERT_EQ(subsystems.size(), 2u);
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
