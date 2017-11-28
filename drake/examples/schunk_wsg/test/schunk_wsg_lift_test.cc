/// @file
///
/// This file implements a test that the Schunk WSG50 gripper can grip
/// a box which is sitting on the world, raise it into the air, and
/// not drop the box.
///
/// This test is, in some sense, a warning alarm.  Given the implemented contact
/// model and the encoded parameters, this test implicitly defines an acceptable
/// *behavior*.  It doesn't test the contact model's functionality (the contact
/// model has its own unit tests.)  It also doesn't guarantee physical
/// correctness. Instead, it evaluates the contact model in a larger integrated
/// context, quantifies "valid" behavior, in some sense, and serves as an early
/// warning that can indicate if something has changed in the system such that
/// the final system no longer reproduces the expected baseline behavior.

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/kinematics_results.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/trajectory_source.h"

namespace drake {
namespace examples {
namespace schunk_wsg {
namespace {

using drake::systems::RungeKutta3Integrator;
using drake::systems::ContactResultsToLcmSystem;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::KinematicsResults;
using Eigen::Vector3d;

// Initial height of the box's origin.
const double kBoxInitZ = 0.076;

std::unique_ptr<RigidBodyTreed> BuildLiftTestTree(
    int* lifter_instance_id, int* gripper_instance_id) {
  std::unique_ptr<RigidBodyTreed> tree = std::make_unique<RigidBodyTreed>();
  multibody::AddFlatTerrainToWorld(tree.get());

  // Add a joint to the world which can lift the gripper.
  const auto lifter_id_table =
      parsers::sdf::AddModelInstancesFromSdfFile(
      FindResourceOrThrow("drake/examples/schunk_wsg/test/test_lifter.sdf"),
      multibody::joints::kFixed, nullptr, tree.get());
  EXPECT_EQ(lifter_id_table.size(), 1);
  *lifter_instance_id = lifter_id_table.begin()->second;

  /// Add the gripper.  Offset it slightly back and up so that we can
  // locate the target at the origin.
  auto gripper_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "lifted_link_frame",
      tree->FindBody("lifted_link"), Eigen::Vector3d(0, -0.05, 0.05),
      Eigen::Vector3d::Zero());
  const auto gripper_id_table = parsers::sdf::AddModelInstancesFromSdfFile(
      FindResourceOrThrow(
          "drake/manipulation/models/wsg_50_description/sdf/"
          "schunk_wsg_50_ball_contact.sdf"),
      multibody::joints::kFixed, gripper_frame, tree.get());
  EXPECT_EQ(gripper_id_table.size(), 1);
  *gripper_instance_id = gripper_id_table.begin()->second;

  // Add a box to grip.
  auto box_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world",
      nullptr,
      Eigen::Vector3d(0, 0, kBoxInitZ), Eigen::Vector3d::Zero());
  parsers::urdf::AddModelInstanceFromUrdfFile(
      FindResourceOrThrow("drake/multibody/models/box_small.urdf"),
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
  plant->set_name("plant");

  ASSERT_EQ(plant->get_num_actuators(), 2);
  ASSERT_EQ(plant->get_num_model_instances(), 3);

  // Arbitrary contact parameters.
  const double kYoungsModulus = 1e8;  // Pa
  const double kDissipation = 2.0;  // s/m
  const double kStaticFriction = 0.9;
  const double kDynamicFriction = 0.5;
  systems::CompliantMaterial default_material;
  default_material.set_youngs_modulus(kYoungsModulus)
      .set_dissipation(kDissipation)
      .set_friction(kStaticFriction, kDynamicFriction);
  plant->set_default_compliant_material(default_material);

  const double kVStictionTolerance = 0.01;  // m/s
  const double kContactArea = 2e-4;  // m^2
  systems::CompliantContactModelParameters model_parameters;
  model_parameters.characteristic_area = kContactArea;
  model_parameters.v_stiction_tolerance = kVStictionTolerance;
  plant->set_contact_model_parameters(model_parameters);

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
      systems::controllers::PidControlledSystem<double>::ConnectController(
          lifting_input_port, lifting_output_port,
          lift_kp, lift_ki, lift_kd, &builder);

  auto zero_source =
      builder.AddSystem<systems::ConstantVectorSource<double>>(
          Eigen::VectorXd::Zero(1));
  zero_source->set_name("zero");
  builder.Connect(zero_source->get_output_port(),
                  lifting_pid_ports.control_input_port);

  // Create a trajectory with 3 seconds of main lift time from second
  // 1 to second 4, coming to a stop by second 5.  The initial delay
  // is to allow the gripper to settle on the box before we start
  // moving.
  const double kLiftHeight = 1.0;
  const double kLiftStart = 1.0;
  std::vector<double> lift_breaks{0., 0.9, kLiftStart, 4., 5};
  std::vector<Eigen::MatrixXd> lift_knots;
  lift_knots.push_back(Eigen::Vector2d(0., 0.));
  lift_knots.push_back(Eigen::Vector2d(0., 0.));
  lift_knots.push_back(Eigen::Vector2d(0., 0.09 * kLiftHeight));
  lift_knots.push_back(Eigen::Vector2d(0.9 * kLiftHeight, 0.09 * kLiftHeight));
  // Stop gradually.
  lift_knots.push_back(Eigen::Vector2d(kLiftHeight, 0.));
  PiecewisePolynomialTrajectory lift_trajectory(
      PiecewisePolynomial<double>::Cubic(
          lift_breaks, lift_knots, Eigen::Vector2d(0., 0.),
          Eigen::Vector2d(0., 0.)));
  auto lift_source =
      builder.AddSystem<systems::TrajectorySource>(lift_trajectory);
  lift_source->set_name("lift_source");
  builder.Connect(lift_source->get_output_port(),
                  lifting_pid_ports.state_input_port);

  // Create a trajectory for grip force.
  // Settle the grip by the time the lift starts.
  std::vector<double> grip_breaks{0., kLiftStart - 0.1, kLiftStart};
  std::vector<Eigen::MatrixXd> grip_knots;
  grip_knots.push_back(Vector1d(0));
  grip_knots.push_back(Vector1d(0));
  grip_knots.push_back(Vector1d(40));
  PiecewisePolynomialTrajectory grip_trajectory(
      PiecewisePolynomial<double>::FirstOrderHold(grip_breaks, grip_knots));
  auto grip_source =
      builder.AddSystem<systems::TrajectorySource>(grip_trajectory);
  grip_source->set_name("grip_source");
  builder.Connect(grip_source->get_output_port(),
                  plant->model_instance_actuator_command_input_port(
                      gripper_instance_id));

  // Creates and adds LCM publisher for visualization.  The test doesn't
  // require `drake_visualizer` but it is convenient to have when debugging.
  drake::lcm::DrakeLcm lcm;
  auto viz_publisher = builder.template AddSystem<systems::DrakeVisualizer>(
      plant->get_rigid_body_tree(), &lcm);
  viz_publisher->set_name("visualization_publisher");
  builder.Connect(plant->state_output_port(),
                  viz_publisher->get_input_port(0));

  // contact force visualization
  ContactResultsToLcmSystem<double>& contact_viz =
      *builder.template AddSystem<ContactResultsToLcmSystem<double>>(
          plant->get_rigid_body_tree());
  contact_viz.set_name("contact_visualization");
  auto& contact_results_publisher = *builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", &lcm));
  contact_results_publisher.set_name("contact_results_publisher");
  // Contact results to lcm msg.
  builder.Connect(plant->contact_results_output_port(),
                  contact_viz.get_input_port(0));
  builder.Connect(contact_viz.get_output_port(0),
                  contact_results_publisher.get_input_port(0));

  const int plant_output_port = builder.ExportOutput(plant->get_output_port(0));
  // Expose the RBPlant kinematics results as a diagram output for body state
  // validation.
  const int kinematrics_results_index =
      builder.ExportOutput(plant->get_output_port(
          plant->kinematics_results_output_port().get_index()));

  // Set up the model and simulator and set their starting state.
  const std::unique_ptr<systems::Diagram<double>> model = builder.Build();
  systems::Simulator<double> simulator(*model);

  const RigidBodyTreed& tree = plant->get_rigid_body_tree();

  // Open the gripper.  Due to the number of links involved, this is
  // surprisingly complicated.
  systems::Context<double>& plant_context =
      model->GetMutableSubsystemContext(
          *plant, &simulator.get_mutable_context());
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
  plant->set_state_vector(&plant_context, plant_initial_state);

  systems::Context<double>& context = simulator.get_mutable_context();

  simulator.reset_integrator<RungeKutta3Integrator<double>>(*model, &context);
  simulator.get_mutable_integrator()->request_initial_step_size_target(1e-4);
  simulator.get_mutable_integrator()->set_target_accuracy(1e-3);

  simulator.Initialize();

  // Simulate to one second beyond the trajectory motion.
  const double kSimDuration = lift_breaks[lift_breaks.size() - 1] + 1.0;

  // Simulation in two pieces -- see notes below on the test for details.
  simulator.StepTo(kLiftStart);

  // Capture the "initial" positions of the box and the gripper finger as
  // discussed in the test notes below.
  auto state_output = model->AllocateOutput(simulator.get_context());
  model->CalcOutput(simulator.get_context(), state_output.get());
  auto& interim_kinematics_results =
      state_output->get_data(kinematrics_results_index)
          ->GetValue<KinematicsResults<double>>();
  const int box_index = tree.FindBodyIndex("box");
  Vector3d init_box_pos =
      interim_kinematics_results.get_body_position(box_index);
  const int finger_index = tree.FindBodyIndex("left_finger");
  Vector3d init_finger_pos =
      interim_kinematics_results.get_body_position(finger_index);

  // Now run to the end of the simulation.
  simulator.StepTo(kSimDuration);

  // Extract and log the state of the robot.
  model->CalcOutput(simulator.get_context(), state_output.get());
  const auto final_output_data =
      state_output->get_vector_data(plant_output_port)->get_value();

  drake::log()->debug("Final state:");
  const int num_movable_links = plant->get_num_positions();
  for (int link_index = 0; link_index < num_movable_links; link_index++) {
    drake::log()->debug("  {} {}: {} (v={})",
                        link_index, tree.get_position_name(link_index),
                        final_output_data[link_index],
                        final_output_data[link_index + num_movable_links]);
  }

  // Expect that the gripper is open (even though clamping force is
  // being applied).
  EXPECT_GT(final_output_data[
      positions["right_finger_sliding_joint"]], 0.02);

  // This is a bound on the expected behavior and implicitly defines what we
  // consider to be "acceptable" stiction behavior.
  //
  // In perfect stiction, once the gripper grabs the box, the box should undergo
  // the exact same transformation as the gripper. To measure this, we've
  // captured the position of the box and one of the fingers at the moment a
  // secure grip is made -- or the best approximation of that moment -- the end
  // of the gripping trajectory.
  //
  // At the end of the simulation, we measure the position of the finger again.
  // We then measure the displacement of the finger and apply that same
  // displacement to the box to represent the ideal stiction position of the
  // box. (NOTE: This assumes that there is no *rotational* displacement on the
  // gripper.)
  //
  // Based on this ideal end position, we compute the difference between ideal
  // and actual final position. The _mean slip speed_ is that difference
  // divided by the simulation duration. Successful simulation means that
  // the box will have slipped at a rate no greater than that specified by
  // kVStictionTolerance.
  //
  // Note that this isn't a *definitive* metric.  It's an attempt at a
  // quantitative metric to indicate that the behavior lies with in an expected
  // qualitative threshold. Ideally, the relative velocities between two
  // contacting bodies *at the point of contact* would be evaluated at
  // every time step. This is an approximation of that appropriate to a big-
  // picture, what-is-the-end-result view.
  //
  // The approximation will introduce some error from various sources:
  //   - We are looking at the overall displacement of the center of mass and
  //     not its actual trajectory (which is more likely an arc).  However, for
  //     small curves, the chord is a reasonable approximation of the arc.
  //   - The stiction model doesn't *actually* guarantee the slip velocity of
  //     the body origin; it guarantees the speed of the contact points. Given
  //     a long enough lever arm, the disparity between speeds at those to
  //     locations can be significant.  However, this is a small scale problem
  //     with multiple points of contact which would help reduce the lever
  //     effect, so this approximation isn't particularly destructive.
  //   - We allow a fair amount of slippage (at a rate of 0.01 m/s for a box
  //     whose scale is on the same order of magnitude).  This is a testing
  //     expediency to allow for timely execution.  This does not *prove* that
  //     the behavior is correct for smaller thresholds (and the corresponding
  //     more precise integrator settings).

  // Compute expected final position and compare with observed final position.
  auto& final_kinematics_results =
      state_output->get_data(kinematrics_results_index)
          ->GetValue<KinematicsResults<double>>();
  Vector3d final_finger_pos =
      final_kinematics_results.get_body_position(finger_index);
  Vector3d ideal_final_pos(init_box_pos);
  ideal_final_pos += final_finger_pos - init_finger_pos;
  Vector3d final_pos = final_kinematics_results.get_body_position(box_index);
  Vector3d displacement = final_pos - ideal_final_pos;
  double distance = displacement.norm();

  // Lift duration is a sub-interval of the full simulation.
  const double kLiftDuration = kSimDuration - kLiftStart;
  const double kMeanSlipSpeed = distance / kLiftDuration;
  EXPECT_LT(kMeanSlipSpeed, kVStictionTolerance);
}

}  // namespace
}  // namespace schunk_wsg
}  // namespace examples
}  // namespace drake
