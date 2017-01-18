#include "drake/systems/sensors/accelerometer.h"

#include <memory>

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/lcm/drake_mock_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/model_instance_id_table.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcmt_drake_signal_translator.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/sensors/accelerometer_output.h"

using Eigen::Vector3d;

using std::make_unique;
using std::move;
using std::string;
using std::stringstream;
using std::unique_ptr;

namespace drake {

using lcm::DrakeMockLcm;
using parsers::sdf::AddModelInstancesFromSdfFileToWorld;
using parsers::urdf::AddModelInstanceFromUrdfFileToWorld;

namespace systems {

using lcm::LcmPublisherSystem;
using lcm::LcmSubscriberSystem;
using lcm::LcmtDrakeSignalTranslator;

namespace sensors {
namespace {

const char* const kSensorName = "foo sensor";

// Tests Accelerometer's various accessor and streaming to-string methods.
GTEST_TEST(TestAccelerometer, AccessorsAndToStringTest) {
  auto tree = make_unique<RigidBodyTree<double>>();
  RigidBodyFrame<double> frame("foo frame", &tree->world(),
                               Eigen::Isometry3d::Identity());

  // Defines the Device Under Test (DUT).
  Accelerometer dut(kSensorName, frame, *tree);

  stringstream string_buffer;
  string_buffer << dut;
  const string dut_string = string_buffer.str();

  EXPECT_NE(dut_string.find("Accelerometer:"), string::npos);
  EXPECT_NE(dut_string.find("name ="), string::npos);
  EXPECT_NE(dut_string.find("frame ="), string::npos);
  EXPECT_EQ(std::count(dut_string.begin(), dut_string.end(), '\n'), 3);
}

// Tests that the accelerometer attached to a single rigid body floating in
// space can measure the effects of gravity.
GTEST_TEST(TestAccelerometer, TestFreeFall) {
  auto tree = make_unique<RigidBodyTree<double>>();

  // Adds a box to the RigidBodyTree and obtains its model instance ID.
  const parsers::ModelInstanceIdTable model_instance_id_table =
      AddModelInstanceFromUrdfFileToWorld(
          GetDrakePath() + "/multibody/models/box.urdf",
          drake::multibody::joints::kQuaternion, tree.get());

  // Adds a frame to the RigidBodyTree called "box frame" that is coincident
  // with the "box" body within the RigidBodyTree.
  auto frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "box frame",
      tree->FindBody("box"), Eigen::Isometry3d::Identity());
  tree->addFrame(frame);
  EXPECT_EQ(tree->get_num_actuators(), 0);

  // Defines the Device Under Test (DUT).
  Accelerometer dut(kSensorName, *frame, *tree);

  unique_ptr<Context<double>> dut_context = dut.CreateDefaultContext();
  EXPECT_EQ(dut_context->get_num_input_ports(), 2);
  EXPECT_EQ(dut_context->get_continuous_state_vector().size(), 0);

  const int num_positions = tree->get_num_positions();
  const int num_velocities = tree->get_num_velocities();
  const int num_states = num_positions + num_velocities;

  // The RigidBodyTree has 13 DOFs: 7 generalized positions and 6 generalized
  // velocities.
  EXPECT_EQ(num_positions, 7);
  EXPECT_EQ(num_velocities, 6);

  dut_context->FixInputPort(
      dut.get_plant_state_input_port().get_index(),
      make_unique<BasicVector<double>>(VectorX<double>::Zero(num_states)));
  dut_context->FixInputPort(
      dut.get_plant_state_derivative_input_port().get_index(),
      make_unique<BasicVector<double>>(VectorX<double>::Zero(num_states)));

  auto xc_vector = make_unique<BasicVector<double>>(
      VectorX<double>::Zero(num_states).eval());
  auto xc = make_unique<ContinuousState<double>>(move(xc_vector), num_positions,
                                                 num_velocities,
                                                 0 /* num other variables */);

  dut_context->set_continuous_state(move(xc));

  unique_ptr<SystemOutput<double>> output = dut.AllocateOutput(*dut_context);
  ASSERT_EQ(output->get_num_ports(), 1);
  dut.CalcOutput(*dut_context, output.get());

  const Vector3d expected_acceleration(0, 0, 9.81);
  EXPECT_TRUE(CompareMatrices(output->get_vector_data(0)->get_value(),
                              expected_acceleration, 1e-10,
                              MatrixCompareType::absolute));
}

// Tests the accelerometer using a non-identity sensor frame that's attached
// to the end of the outboard link of a double pendulum whose based is fixed to
// the world.
GTEST_TEST(TestAccelerometer, TestSensorAttachedToDoublePendulum) {
  auto tree = make_unique<RigidBodyTree<double>>();

  // Adds a box to the RigidBodyTree and obtains its model instance ID.
  const parsers::ModelInstanceIdTable model_instance_id_table =
  AddModelInstancesFromSdfFileToWorld(
          GetDrakePath() + "/multibody/parsers/test/parsers_frames_test/"
              "simple_double_pendulum_l_equals_b_sdf/"
              "simple_double_pendulum_l_equals_b.sdf",
          drake::multibody::joints::kFixed, tree.get());

  // Positions the accelerometer at the surface of the end of the lower arm
  // link.
  Eigen::Isometry3d sensor_frame_transform = Eigen::Isometry3d::Identity();
  sensor_frame_transform.translation() << 0, -1, 0.05;

  // Adds a frame to the RigidBodyTree called "sensor frame" that is coincident
  // with the "box" body within the RigidBodyTree.
  auto sensor_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "sensor frame",
      tree->FindBody("lower_arm"), sensor_frame_transform);
  tree->addFrame(sensor_frame);

  DiagramBuilder<double> builder;
  const RigidBodyPlant<double>* plant =
      builder.template AddSystem<RigidBodyPlant<double>>(std::move(tree));

  LcmtDrakeSignalTranslator translator(plant->get_num_velocities());
  DrakeMockLcm lcm(true /* enable_loop_back */);

  const LcmPublisherSystem* lcm_publisher =
      builder.template AddSystem<LcmPublisherSystem>("xdot_channel", translator,
                                                     &lcm);
  const LcmSubscriberSystem* lcm_subscriber =
      builder.template AddSystem<LcmSubscriberSystem>("xdot_channel",
                                                       translator, &lcm);
  const Accelerometer* accelerometer = Accelerometer::AttachAccelerometer(
      "my accelerometer",
      *sensor_frame,
      *plant,
      true /* include_gravity */,
      &builder);

  auto constant_zero_source =
      builder.template AddSystem<ConstantVectorSource<double>>(
          VectorX<double>::Zero(plant->get_input_port(0).size()));

  builder.Connect(plant->state_output_port(),
                  accelerometer->get_plant_state_input_port());
  builder.Connect(plant->state_output_port(),
                  lcm_publisher->get_input_port(0));
  builder.Connect(lcm_subscriber->get_output_port(0),
                  accelerometer->get_plant_state_derivative_input_port());
  builder.Connect(*constant_zero_source, *plant);
  builder.ExportOutput(accelerometer->get_output_port());
  std::unique_ptr<Diagram<double>> diagram = builder.Build();

  // Prepares to integrate.
  unique_ptr<Context<double>> context = diagram->AllocateContext();
  drake::systems::Simulator<double> simulator(*diagram, std::move(context));
  simulator.set_publish_every_time_step(true);
  simulator.Initialize();

  simulator.StepTo(0.1);

  VectorX<double> accelerometer_measurement = context->get_continuous_state_vector().CopyToVector();
  std::cout <<"Accelerometer measurement: " << accelerometer_measurement.transpose() << std::endl;
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
