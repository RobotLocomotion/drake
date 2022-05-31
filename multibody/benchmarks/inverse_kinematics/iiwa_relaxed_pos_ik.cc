// @file
// Benchmark for InverseKinematics.
//
// This benchmark is intended to analyze the performance of nonlinear
// optimization with position constraints.

#include "drake/common/find_resource.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/inverse_kinematics/position_constraint.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"
#include "drake/tools/performance/fixture_common.h"

namespace drake {
namespace multibody {
namespace inverse_kinematics {
namespace {

class RelaxedPosIkBenchmark : public benchmark::Fixture {
 public:
  RelaxedPosIkBenchmark() {
    tools::performance::AddMinMaxStatistics(this);

    // Set a fixed seed for random generation.
    std::srand(1234);
  }
};

BENCHMARK_F(RelaxedPosIkBenchmark, Iiwa)(benchmark::State& state) {  // NOLINT
  // Solve an inverse kinematics problem for Kuka iiwa considering only the
  // translation component with relaxation. The objective of this benchmark is
  // to run an A/B comparison for different gradient evaluations for
  // position constraints when solving a nonlinear program.

  // Find the model file for Kuka iiwa.
  const std::string iiwa_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/iiwa7/"
      "iiwa7_no_collision.sdf");
  // Create a continuous-time plant.
  multibody::MultibodyPlant<double> plant(0.0);
  // Create a parser for the plant.
  multibody::Parser parser{&plant};
  // Load the model into the parser.
  const multibody::ModelInstanceIndex model_instance =
      parser.AddModelFromFile(iiwa_path);
  // Attach the base of the robot into the world frame.
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("iiwa_link_0", model_instance));
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

  // Define a uniform position relaxation.
  const Eigen::Vector3d pos_tol = 1e-4 * Eigen::Vector3d::Ones();

  // Get the joint position limits for random generation by leveraging the
  // fact that iiwa has symmetric position limits.
  const Eigen::VectorXd joint_pos_limits(plant.GetPositionUpperLimits());
  // Generate random goal configurations.
  const int num_rand_goals = 10;
  std::vector<math::RigidTransformd> ee_pose_goal(num_rand_goals);
  for (int i = 0; i < num_rand_goals; ++i) {
    // Set a random joint pose within 90% of the limits.
    context->get_mutable_continuous_state()
        .get_mutable_generalized_position()
        .SetFromVector(
            Eigen::VectorXd::Random(7).cwiseProduct(0.9 * joint_pos_limits));
    // Evaluate the corresponding end-effector pose.
    ee_pose_goal[i] = plant.EvalBodyPoseInWorld(*context, ee_body);
  }

  // Sample random initial guesses assuming that the range [-1, 1] rad
  // does not violate any of the joint position limits.
  const int num_rand_init_guess = 3;
  const Eigen::MatrixXd q0(Eigen::MatrixXd::Random(7, num_rand_init_guess));

  // Create the task in terms of position constraints on the end effector.
  auto pos_constraint = std::make_shared<PositionConstraint>(
      &plant, plant.world_frame(), -pos_tol, pos_tol, ee_frame,
      Eigen::Vector3d::Zero(), context.get());
  prog->AddConstraint(pos_constraint, relaxed_ik.q());

  for (auto _ : state) {
    // Solve the problem for each goal and initial guess.
    for (int i = 0; i < num_rand_goals; ++i) {
      // Update the task constraint.
      pos_constraint->set_bounds(ee_pose_goal[i].translation() - pos_tol,
                                 ee_pose_goal[i].translation() + pos_tol);

      for (int j = 0; j < num_rand_init_guess; ++j) {
        // Set the initial guess.
        prog->SetInitialGuess(relaxed_ik.q(), q0.col(j));

        // Solve the problem.
        auto result = solvers::Solve(*prog);

        // Confirm that the optimization has succeeded.
        // This is done only for SNOPT b/c IPOPT exceeds the maximum number of
        // iterations in a few cases while still meeting the task constraints.
        if (result.get_solver_id() == solvers::SnoptSolver::id())
          DRAKE_DEMAND(result.is_success());

        // Check whether the task constraint is met by providing extra margin to
        // account for solver tolerances.
        DRAKE_DEMAND(
            pos_constraint->CheckSatisfied(result.GetSolution(), 1e-5));
      }
    }
  }
}

}  // namespace
}  // namespace inverse_kinematics
}  // namespace multibody
}  // namespace drake

BENCHMARK_MAIN();
