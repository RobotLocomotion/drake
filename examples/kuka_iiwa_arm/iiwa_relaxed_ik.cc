#include <gflags/gflags.h>
#include <iostream>

#include "drake/common/find_resource.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/inverse_kinematics/position_constraint.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/solver_options.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

int DoMain() {
  // Solve an inverse kinematics problem for Kuka iiwa considering only the
  // translation component with relaxation.

  // Define constants.
  // A uniform relaxation for end-effector position.
  const Eigen::Vector3d kPosTol = 1e-3 * Eigen::Vector3d::Ones();
  // The number of random goals.
  const int kNumRandGoals = 1;
  // The number of random initial guesses for each goal.
  const int kNumRandInitGuess = 1;

  // Find the model file for Kuka iiwa.
  const std::string iiwa_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/iiwa7/"
      "iiwa7_no_collision.sdf");
  // Create a continuous-time plant.
  multibody::MultibodyPlant<double> plant(0.0);
  // Create a parser for the plant.
  multibody::Parser parser{&plant};
  // Load the model into the parser.
  parser.AddModels(iiwa_path);
  // Attach the base of the robot into the world frame.
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("iiwa_link_0"));
  // Finalize the plant.
  plant.Finalize();
  // Create a context.
  std::unique_ptr<systems::Context<double>> context =
      plant.CreateDefaultContext();

  // Define the end-effector link and get the corresponding body and frame.
  const std::string ee_link_name = "iiwa_link_7";
  const multibody::Body<double>& ee_body = plant.GetBodyByName(ee_link_name);
  const multibody::Frame<double>& ee_frame = plant.GetFrameByName(ee_link_name);

  // Create an IK object and get its mathetical program.
  multibody::InverseKinematics relaxed_ik(plant, true);
  solvers::MathematicalProgram* prog = relaxed_ik.get_mutable_prog();

  // Create solver and set options.
  solvers::SnoptSolver snopt;
  solvers::SolverOptions sol_opt;
  // sol_opt.SetOption(solvers::CommonSolverOption::kPrintFileName, "snopt_test.out");
  sol_opt.SetOption(solvers::CommonSolverOption::kPrintToConsole, 1);
  prog->SetSolverOptions(sol_opt);
  prog->SetSolverOption(solvers::SnoptSolver::id(), "Print file", "snopt_test.out");

  // Generate random goal configurations.
  std::vector<math::RigidTransformd> ee_pose_goal(kNumRandGoals);
  for (int i = 0; i < kNumRandGoals; ++i) {
    // Sample a random joint pose assuming that the random range [-1, 1] rad
    // does not violate any of the joint position limits.
    context->get_mutable_continuous_state()
        .get_mutable_generalized_position()
        .SetFromVector(Eigen::VectorXd::Ones(plant.num_positions()));
    // Evaluate the corresponding end-effector pose.
    ee_pose_goal[i] = plant.EvalBodyPoseInWorld(*context, ee_body);
  }

  // Sample random initial guesses assuming that the random range [-1, 1] rad
  // does not violate any of the joint position limits.
  const Eigen::MatrixXd q0(
      1e-1 * Eigen::MatrixXd::Ones(plant.num_positions(), kNumRandInitGuess));

  // Create the task in terms of position constraints on the end effector.
  auto pos_constraint = std::make_shared<multibody::PositionConstraint>(
      &plant, plant.world_frame(), -kPosTol, kPosTol, ee_frame,
      Eigen::Vector3d::Zero(), context.get());
  prog->AddConstraint(pos_constraint, relaxed_ik.q());

  // Solve the problem for each goal and initial guess.
  for (int i = 0; i < kNumRandGoals; ++i) {
    // Update the task constraint.
    pos_constraint->set_bounds(ee_pose_goal[i].translation() - kPosTol,
                               ee_pose_goal[i].translation() + kPosTol);

    for (int j = 0; j < kNumRandInitGuess; ++j) {
      // Set the initial guess.
      prog->SetInitialGuess(relaxed_ik.q(), q0.col(j));
      std::cout << "\n\tq0: ";
      for (int xi=0; xi<q0.size(); xi++)
        std::cout << q0(xi) << " ";
      std::cout << std::endl;

      // Solve the problem.
      std::cout << "\nRef. pose:\n" << ee_pose_goal[i] << "\n";
      auto result = snopt.Solve(*prog);

      auto x_val = result.GetSolution();

      std::cout << "\n\tSolution: ";
      for (int xi=0; xi<x_val.size(); xi++)
        std::cout << x_val[xi] << " ";
      std::cout << std::endl;

      // Evaluate the corresponding end-effector pose.
      context->get_mutable_continuous_state()
          .get_mutable_generalized_position()
          .SetFromVector(x_val);
      auto ee_pose = plant.EvalBodyPoseInWorld(*context, ee_body);
      std::cout << "\nAct. pose:\n" << ee_pose << "\n";
    }
  }

  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::DoMain();
}
