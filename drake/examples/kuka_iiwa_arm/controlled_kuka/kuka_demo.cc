#include <iostream>
#include <memory>

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/gravity_compensator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/demultiplexer.h"
#include "drake/systems/framework/primitives/trajectory_source.h"
#include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/plants/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/trajectories/piecewise_polynomial_trajectory.h"

// Includes for the planner.
#include "drake/systems/plants/IKoptions.h"
#include "drake/systems/plants/RigidBodyIK.h"

DEFINE_double(simulation_sec, 0.5, "Number of seconds to simulate.");

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::MatrixXd;
using std::make_unique;
using std::move;
using std::unique_ptr;

namespace drake {

using systems::Context;
using systems::Demultiplexer;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::GravityCompensator;
using systems::PidControlledSystem;
using systems::RigidBodyPlant;
using systems::Simulator;
using systems::TrajectorySource;

namespace examples {
namespace kuka_iiwa_arm {
namespace controlled_kuka {
namespace {

unique_ptr<PiecewisePolynomialTrajectory> MakePlan() {
  RigidBodyTree tree(
      drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
      drake::systems::plants::joints::kFixed);

  // Creates a basic pointwise IK trajectory for moving the iiwa arm.
  // It starts in the zero configuration (straight up).
  VectorXd zero_conf = tree.getZeroConfiguration();
  VectorXd joint_lb = zero_conf - VectorXd::Constant(7, 0.01);
  VectorXd joint_ub = zero_conf + VectorXd::Constant(7, 0.01);

  PostureConstraint pc1(&tree, Vector2d(0, 0.5));
  VectorXi joint_idx(7);
  joint_idx << 0, 1, 2, 3, 4, 5, 6;
  pc1.setJointLimits(joint_idx, joint_lb, joint_ub);

  // Defines an end effector constraint and makes it active for the time span
  // from 1 to 3 seconds.
  Vector3d pos_end;
  pos_end << 0.6, 0, 0.325;
  Vector3d pos_lb = pos_end - Vector3d::Constant(0.005);
  Vector3d pos_ub = pos_end + Vector3d::Constant(0.005);
  WorldPositionConstraint wpc1(
      &tree, tree.FindBodyIndex("iiwa_link_ee"),
      Vector3d::Zero(), pos_lb, pos_ub, Vector2d(1, 3));

  // After the end effector constraint is released, applies the straight
  // up configuration again from time 4 to 5.9.
  PostureConstraint pc2(&tree, Vector2d(4, 5.9));
  pc2.setJointLimits(joint_idx, joint_lb, joint_ub);

  // Apply the same end effector constraint from time 6 to 9 of the demo.
  WorldPositionConstraint wpc2(&tree, tree.FindBodyIndex("iiwa_link_ee"),
                               Vector3d::Zero(), pos_lb, pos_ub,
                               Vector2d(6, 9));

  // For part of the time wpc2 is active, constrains the second joint while
  // preserving the end effector constraint.
  //
  // Variable `joint_position_start_idx` below is a collection of offsets into
  // the state vector referring to the positions of the joints to be
  // constrained.
  Eigen::VectorXi joint_position_start_idx(1);
  joint_position_start_idx(0) = tree.FindChildBodyOfJoint("iiwa_joint_2")->
      get_position_start_index();
  PostureConstraint pc3(&tree, Vector2d(6, 8));
  pc3.setJointLimits(joint_position_start_idx, Vector1d(0.7), Vector1d(0.8));

  // TODO(naveenoid): Explain here in a comment why kNumTimesteps = 5.
  const int kNumTimesteps = 5;
  double t[kNumTimesteps] = { 0.0, 2.0, 5.0, 7.0, 9.0 };
  MatrixXd q0(tree.get_num_positions(), kNumTimesteps);
  for (int i = 0; i < kNumTimesteps; ++i) {
    q0.col(i) = zero_conf;
  }

  std::vector<RigidBodyConstraint*> constraint_array;
  constraint_array.push_back(&pc1);
  constraint_array.push_back(&wpc1);
  constraint_array.push_back(&pc2);
  constraint_array.push_back(&pc3);
  constraint_array.push_back(&wpc2);
  IKoptions ikoptions(&tree);
  int info[kNumTimesteps];
  MatrixXd q_sol(tree.get_num_positions(), kNumTimesteps);
  std::vector<std::string> infeasible_constraint;

  inverseKinPointwise(&tree, kNumTimesteps, t, q0, q0, constraint_array.size(),
                      constraint_array.data(), ikoptions, &q_sol, info,
                      &infeasible_constraint);
  bool info_good = true;
  for (int i = 0; i < kNumTimesteps; ++i) {
    drake::log()->info("INFO[{}] = {} ", i, info[i]);
    if (info[i] != 1) {
      info_good = false;
    }
  }
  printf("\n");

  if (!info_good) {
    throw std::runtime_error(
        "inverseKinPointwise failed to compute a valid solution.");
  }

  // This code comes from TrajectoryRunner in kuka_id_demo.
  // TODO(naveenoid): Remove duplicated code by refactoring TrajectoryRunner
  // out of kuka_ik_demo.
  typedef PiecewisePolynomial<double> PPType;
  typedef PPType::PolynomialType PPPoly;
  typedef PPType::PolynomialMatrix PPMatrix;
  std::vector<PPMatrix> polys;
  std::vector<double> times;

  MatrixXd traj(tree.get_num_positions() + tree.get_num_velocities(),
                 kNumTimesteps);
  traj.setZero();
  traj.block(0, 0, tree.get_num_positions(), kNumTimesteps) = q_sol;

  // For each timestep, creates a PolynomialMatrix for each joint position.
  // Each column of traj represents a particular time, and the rows of that
  // column contain values for each joint coordinate.
  for (int i = 0; i < kNumTimesteps; ++i) {
    PPMatrix poly_matrix(traj.rows(), 1);
    const auto traj_now = traj.col(i);

    // Produces interpolating polynomials for each joint coordinate.
    if (i != kNumTimesteps - 1) {
      for (int row = 0; row < traj.rows(); ++row) {
        Eigen::Vector2d coeffs(0, 0);
        coeffs[0] = traj_now(row);
        // Sets the coefficient such that it will reach the value of
        // the next timestep at the time when we advance to the next
        // piece.  In the event that we're at the end of the
        // trajectory, this will be left 0.
        coeffs[1] = (traj(row, i + 1) - coeffs[0]) / (t[i + 1] - t[i]);
        poly_matrix(row) = PPPoly(coeffs);
      }
      polys.push_back(poly_matrix);
    }
    times.push_back(t[i]);
  }
  auto pp_traj = make_unique<PiecewisePolynomialTrajectory>(
      PPType(polys, times));
  return pp_traj;
}

// A model of a Kuka iiwa arm with position control using gravity compensation
// and a PID controller. A plan is created by the inverse kinematics method
// MakePlan() above and is fed as the desired target using a
// TimeVaryingPolynomialSource system.
template<typename T>
class KukaDemo : public systems::Diagram<T> {
 public:
  KukaDemo() {
    this->set_name("KukaDemo");

    // Instantiates an Multibody Dynamics (MBD) model of the world.
    auto rigid_body_tree = make_unique<RigidBodyTree>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        drake::GetDrakePath() +
        "/examples/kuka_iiwa_arm/urdf/iiwa14_no_collision.urdf",
        drake::systems::plants::joints::kFixed,
        nullptr /* weld to frame */, rigid_body_tree.get());

    AddGround(rigid_body_tree.get());
    VerifyIiwaTree(*rigid_body_tree);

    DiagramBuilder<T> builder;

    // Instantiates a RigidBodyPlant from an MBD model of the world.
    std::unique_ptr<RigidBodyPlant<T>> plant =
        make_unique<RigidBodyPlant<T>>(move(rigid_body_tree));
    plant_ = plant.get();

    DRAKE_ASSERT(plant_->get_input_port(0).get_size() ==
                 plant_->get_num_positions());

    // Create and add PID controller.
    // Constants are chosen by trial and error to qualitatively match an
    // experimental run with the same initial conditions and planner.
    // Quantitative comparisons would require torque control and a more careful
    // estimation of the model constants such as friction in the joints.
    const double kp = 2.0;  // proportional constant.
    const double ki = 0.0;  // integral constant.
    const double kd = 1.0;  // derivative constant.
    controller_ = builder.template AddSystem<PidControlledSystem<T>>(
        std::move(plant), kp, ki, kd);

    gravity_compensator_ = builder.template AddSystem<GravityCompensator<T>>(
        plant_->get_rigid_body_tree());

    // Split the input state into two signals one with the positions and one
    // with the velocities.
    // For Kuka:
    // -  get_num_states() = 14
    // -  get_num_positions() = 7
    rbp_state_demux_ = builder.template AddSystem<Demultiplexer<T>>(
        plant_->get_num_states(), plant_->get_num_positions());

    // Creates a plan and wraps it into a source system.
    poly_trajectory_ = MakePlan();
    desired_plan_ = builder.template AddSystem<TrajectorySource<T>>(
        *poly_trajectory_);

    // Creates and adds LCM publisher for visualization.
    viz_publisher_ = builder.template AddSystem<DrakeVisualizer>(
        plant_->get_rigid_body_tree(), &lcm_);

    // Generates an error signal for the PID controller by subtracting the
    // desired plan state from the RigidBodyPlant's (iiwa arm) state.
    builder.Connect(desired_plan_->get_output_port(0),
                    controller_->get_input_port(1));

    // Splits the RBP output into positions (q) and velocities (v).
    builder.Connect(controller_->get_output_port(0),
                    rbp_state_demux_->get_input_port(0));

    // Connects the gravity compensator to the output generalized positions.
    builder.Connect(rbp_state_demux_->get_output_port(0),
                    gravity_compensator_->get_input_port(0));
    builder.Connect(gravity_compensator_->get_output_port(0),
                    controller_->get_input_port(0));

    // Connects to publisher for visualization.
    builder.Connect(controller_->get_output_port(0),
                    viz_publisher_->get_input_port(0));

    builder.ExportOutput(controller_->get_output_port(0));
    builder.BuildInto(this);
  }

  const RigidBodyPlant<T>& get_kuka_plant() const { return *plant_; }

  Context<T>* get_kuka_context(Context<T>* context) const {
    Context<T>* controller_context =
        this->GetMutableSubsystemContext(context, controller_);
    return controller_->GetMutableSubsystemContext(controller_context, plant_);
  }

  void SetDefaultState(Context<T>* context) const {
    Context<T>* controller_context =
        this->GetMutableSubsystemContext(context, controller_);
    controller_->SetDefaultState(controller_context);

    Context<T>* plant_context =
        controller_->GetMutableSubsystemContext(controller_context, plant_);
    plant_->SetZeroConfiguration(plant_context);
  }

 private:
  RigidBodyPlant<T>* plant_{nullptr};
  PidControlledSystem<T>* controller_{nullptr};
  Demultiplexer<T>* rbp_state_demux_{nullptr};
  GravityCompensator<T>* gravity_compensator_{nullptr};
  TrajectorySource<T>* desired_plan_{nullptr};
  std::unique_ptr<PiecewisePolynomialTrajectory> poly_trajectory_;
  DrakeVisualizer* viz_publisher_{nullptr};
  drake::lcm::DrakeLcm lcm_;
};

int DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);

  KukaDemo<double> model;
  Simulator<double> simulator(model);
  Context<double>* context = simulator.get_mutable_context();
  // Zeroes the state and initializes controller state.
  model.SetDefaultState(context);

  VectorX<double> desired_state = VectorX<double>::Zero(14);
  model.get_kuka_plant().set_state_vector(
      model.get_kuka_context(context), desired_state);

  simulator.Initialize();

  // Simulate for 20 seconds.
  simulator.StepTo(FLAGS_simulation_sec);

  return 0;
}

}  // namespace
}  // namespace controlled_kuka
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::controlled_kuka::DoMain();
}
