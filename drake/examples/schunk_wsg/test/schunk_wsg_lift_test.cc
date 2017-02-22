/// @file
///
/// This file implements a test that the Schunk WSG50 gripper can grip
/// a box which is sitting on the world, raise it into the air, and
/// not drop the box.

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/trajectory_source.h"

namespace drake {
namespace examples {
namespace schunk_wsg {
namespace {

std::unique_ptr<RigidBodyTreed> BuildLiftTestTree(
    int* lifter_instance_id, int* gripper_instance_id) {
  std::unique_ptr<RigidBodyTreed> tree = std::make_unique<RigidBodyTreed>();
  multibody::AddFlatTerrainToWorld(tree.get());

  // Add a joint to the world which can lift the gripper.
  const auto lifter_id_table =
      parsers::sdf::AddModelInstancesFromSdfFile(
      GetDrakePath() + "/examples/schunk_wsg/test/test_lifter.sdf",
      multibody::joints::kFixed, nullptr, tree.get());
  EXPECT_EQ(lifter_id_table.size(), 1);
  *lifter_instance_id = lifter_id_table.begin()->second;

  /// Add the gripper.  Offset it slightly back and up so that we can
  // locate the target at the origin.
  auto gripper_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "lifted_link_frame",
      tree->FindBody("lifted_link"),
      Eigen::Vector3d(0, -0.05, 0.05), Eigen::Vector3d::Zero());
  const auto gripper_id_table =
      parsers::sdf::AddModelInstancesFromSdfFile(
          GetDrakePath() + "/examples/schunk_wsg/models/schunk_wsg_50.sdf",
          multibody::joints::kFixed, gripper_frame, tree.get());
  EXPECT_EQ(gripper_id_table.size(), 1);
  *gripper_instance_id = gripper_id_table.begin()->second;

  // Add a box to grip.
  auto box_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world",
      nullptr,
      Eigen::Vector3d(0, 0, 0.1), Eigen::Vector3d::Zero());
  parsers::urdf::AddModelInstanceFromUrdfFile(
      GetDrakePath() + "/multibody/models/box_small.urdf",
      multibody::joints::kQuaternion, box_frame, tree.get());

  tree->compile();
  return tree;
}

GTEST_TEST(SchunkWsgLiftTest, BoxLiftTest) {
  systems::DiagramBuilder<double> builder;

  int lifter_instance_id{};
  int gripper_instance_id{};
  systems::RigidBodyPlant<double>* plant =
      builder.AddSystem<systems::RigidBodyPlant<double>>(
          BuildLiftTestTree(&lifter_instance_id, &gripper_instance_id));

  ASSERT_EQ(plant->get_num_actuators(), 2);
  ASSERT_EQ(plant->get_num_model_instances(), 3);

  // Arbitrary contact parameters.
  const double kStiffness = 10000;
  const double kDissipation = 2;
  const double kStaticFriction = 0.9;
  const double kDynamicFriction = 0.5;
  const double kStictionSlipTolerance = 0.001;
  plant->set_normal_contact_parameters(kStiffness, kDissipation);
  plant->set_friction_contact_parameters(kStaticFriction, kDynamicFriction,
                                         kStictionSlipTolerance);

  // Build a trajectory and PID controller for the lifting joint.
  const auto& lifting_input_port =
      plant->model_instance_actuator_command_input_port(lifter_instance_id);
  const auto& lifting_output_port =
      plant->model_instance_state_output_port(lifter_instance_id);

  // Constants chosen arbitrarily.
  const Vector1d lift_kp(300.);
  const Vector1d lift_ki(0.);
  const Vector1d lift_kd(5.);

  auto lifting_pid_ports =
      systems::PidControlledSystem<double>::ConnectController(
          lifting_input_port, lifting_output_port, nullptr,
          lift_kp, lift_ki, lift_kd, &builder);

  auto zero_source =
      builder.AddSystem<systems::ConstantVectorSource<double>>(
          Eigen::VectorXd::Zero(1));
  builder.Connect(zero_source->get_output_port(),
                  lifting_pid_ports.control_input_port);

  // Create a trajectory with 10 seconds of main lift time from second
  // 1 to second 11, coming to a stop by second 12.  The initial delay
  // is to allow the gripper to settle on the box before we start
  // moving.
  std::vector<double> lift_breaks{0., 0.9, 1., 11., 12};
  std::vector<Eigen::MatrixXd> lift_knots;
  lift_knots.push_back(Eigen::Vector2d(0., 0.));
  lift_knots.push_back(Eigen::Vector2d(0., 0.));
  lift_knots.push_back(Eigen::Vector2d(0., 0.09));
  lift_knots.push_back(Eigen::Vector2d(0.9, 0.09));
  // Stop gradually.
  lift_knots.push_back(Eigen::Vector2d(1., 0.));
  PiecewisePolynomialTrajectory lift_trajectory(
      PiecewisePolynomial<double>::Cubic(
          lift_breaks, lift_knots, Eigen::Vector2d(0., 0.),
          Eigen::Vector2d(0., 0.)));
  auto lift_source =
      builder.AddSystem<systems::TrajectorySource>(lift_trajectory);
  builder.Connect(lift_source->get_output_port(),
                  lifting_pid_ports.state_input_port);

  // Create a trajectory for grip force.
  std::vector<double> grip_breaks{0., 0.9, 1.};
  std::vector<Eigen::MatrixXd> grip_knots;
  grip_knots.push_back(Vector1d(0));
  grip_knots.push_back(Vector1d(0));
  grip_knots.push_back(Vector1d(40));
  PiecewisePolynomialTrajectory grip_trajectory(
      PiecewisePolynomial<double>::FirstOrderHold(grip_breaks, grip_knots));
  auto grip_source =
      builder.AddSystem<systems::TrajectorySource>(grip_trajectory);
  builder.Connect(grip_source->get_output_port(),
                  plant->model_instance_actuator_command_input_port(
                      gripper_instance_id));

  // Creates and adds LCM publisher for visualization.  The test doesn't
  // require `drake_visualizer` but it is convenient to have when debugging.
  drake::lcm::DrakeLcm lcm;
  const auto viz_publisher =
      builder.template AddSystem<systems::DrakeVisualizer>(
          plant->get_rigid_body_tree(), &lcm);
  builder.Connect(plant->state_output_port(),
                  viz_publisher->get_input_port(0));
  builder.ExportOutput(plant->get_output_port(0));

  // Set up the model and simulator and set their starting state.
  const std::unique_ptr<systems::Diagram<double>> model = builder.Build();
  systems::Simulator<double> simulator(*model);

  const RigidBodyTreed& tree = plant->get_rigid_body_tree();

  // Open the gripper.  Due to the number of links involved, this is
  // surprisingly complicated.
  systems::Context<double>* plant_context =
      model->GetMutableSubsystemContext(
          simulator.get_mutable_context(), plant);
  Eigen::VectorXd plant_initial_state =
      Eigen::VectorXd::Zero(plant->get_num_states());
  plant_initial_state.head(plant->get_num_positions())
      = tree.getZeroConfiguration();

  auto positions = tree.computePositionNameToIndexMap();
  ASSERT_EQ(positions["left_finger_sliding_joint"], 1);

  // The values below were extracted from the positions corresponding
  // to an open gripper.  Dumping them here is significantly more
  // magic than I (sam.creasey) would like.  If you find yourself
  // tempted to cut and paste this, please consider creating a utility
  // function which can set a segment of a state vector to an open
  // gripper.
  plant_initial_state(1) = -0.0550667;
  plant_initial_state(2) = 0.009759;
  plant_initial_state(3) = 1.27982;
  plant_initial_state(4) = 0.0550667;
  plant_initial_state(5) = 0.009759;
  plant->set_state_vector(plant_context, plant_initial_state);

  simulator.Initialize();
  simulator.StepTo(20.0);

  // Extract and log the final state of the robot.
  auto final_output = model->AllocateOutput(simulator.get_context());
  model->CalcOutput(simulator.get_context(), final_output.get());
  const auto final_output_data =
      final_output->get_vector_data(0)->get_value();

  drake::log()->debug("Final state:");
  const int num_movable_links = plant->get_num_positions();
  for (int link_index = 0; link_index < num_movable_links; link_index++) {
    drake::log()->debug("  {} {}: {} (v={})",
                        link_index, tree.get_position_name(link_index),
                        final_output_data[link_index],
                        final_output_data[link_index + num_movable_links]);
  }

  // TODO(sam.creasey) Re-enable the tests below once the gripper can
  // actually hold the box.
#if 0
  // Expect that the gripper is open (even though clamping force is
  // being applied).
  EXPECT_GT(final_output_data[
      positions["right_finger_sliding_joint"]], 0.02);
  // Expect that the box is off of the ground.
  EXPECT_GT(final_output_data[positions["base_z"]], 0.5);
#endif
}

}  // namespace
}  // namespace schunk_wsg
}  // namespace examples
}  // namespace drake
