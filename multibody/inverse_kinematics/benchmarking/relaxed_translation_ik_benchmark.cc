// @file
// Benchmarks for ConstraintRelaxingIk.
//
// This benchmark is intended to analyze the performance of nonlinear
// optimization with position constraints.

#include <iostream>

#include "drake/common/find_resource.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/binding.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"
#include "drake/tools/performance/fixture_common.h"

static void BenchmarkRelaxedIk(benchmark::State& state) {
  // Formulate an inverse kinematics problem for Kuka iiwa considering only the
  // translational component with relaxation. The objective of this benchmark is
  // to run an A/B comparison for different gradient evaluations for the
  // position constraints.
  for (auto _ : state) {
    // Find the model file for Kuka iiwa.
    const std::string iiwa_path = drake::FindResourceOrThrow(
        "drake/manipulation/models/iiwa_description/iiwa7/"
        "iiwa7_no_collision.sdf");
    // Create a continuous-time plant.
    drake::multibody::MultibodyPlant<double> plant(0.0);
    // Create a parser for the plant.
    drake::multibody::Parser parser{&plant};
    //   Load the model into the parser.
    const drake::multibody::ModelInstanceIndex model_instance =
        parser.AddModelFromFile(iiwa_path);
    // Attach the base of the robot into the world frame.
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("iiwa_link_0", model_instance));
    // Finalize the plant.
    plant.Finalize();
    // Create a context.
    std::unique_ptr<drake::systems::Context<double>> context =
        plant.CreateDefaultContext();

    // Define the end-effector link.
    const std::string ee_link_name = "iiwa_link_7";
    const drake::multibody::Body<double>& ee_body =
        plant.GetBodyByName(ee_link_name);
    const drake::multibody::Frame<double>& ee_frame =
        plant.GetFrameByName(ee_link_name);

    // Get the joint position limits for random generation.

    // Create an IK object.
    drake::multibody::InverseKinematics relaxed_ik(plant, true);
    drake::solvers::MathematicalProgram* prog = relaxed_ik.get_mutable_prog();

    // Define a uniform position relaxation and the position bound variables.
    Eigen::Vector3d pos_tol = 1e-3 * Eigen::Vector3d::Ones();

    // Define generalized position vectors for the goal, inital guess, and the
    // solution.
    Eigen::VectorXd q_goal(7), q0(7), q_sol(7);
    // Define the end-effector poses corresponding to the goal and the solution.
    drake::math::RigidTransformd ee_pose_goal, ee_pose_sol;
    // Define a variable for the mathematical program result.
    drake::solvers::MathematicalProgramResult result;

    // Set a fixed seed for random generation.
    std::srand(1234);
    // Generate 10 goals randomly.
    for (int i = 0; i < 10; ++i) {
      context->get_mutable_continuous_state()
          .get_mutable_generalized_position()
          .SetFromVector(Eigen::VectorXd::Random(7));
      std::cout << "\trandom pos: " << plant.GetPositions(*context).transpose()
                << '\n';
      // Evaluate the corresponding end-effector position.
      ee_pose_goal = plant.EvalBodyPoseInWorld(*context, ee_body);

      // If not the first iteration, remove the position constraint.
      if (i > 0) {
        auto constraints = prog->GetAllConstraints();
        for (auto constraint : constraints) {
          prog->RemoveConstraint(constraint);
        }
      }
      // Update the task otherwise.
      else {
        relaxed_ik.AddPositionConstraint(ee_frame, Eigen::Vector3d::Zero(),
                                         plant.world_frame(),
                                         ee_pose_goal.translation() - pos_tol,
                                         ee_pose_goal.translation() + pos_tol);
      }

      // Solve each task using three random initial guesses.
      for (int j = 0; j < 3; ++j) {
        // Sample a random initial guess.
        q0 = Eigen::VectorXd::Random(7);

        // Solve the relaxed IK problem.
        result = drake::solvers::Solve(*prog);
        // Get the solution
        q_sol = result.GetSolution();
        // Evaluate the end-effector pose for the solution.
        context->get_mutable_continuous_state()
            .get_mutable_generalized_position()
            .SetFromVector(q_sol);
        ee_pose_sol = plant.EvalBodyPoseInWorld(*context, ee_body);

        // Print the results.
        std::cout << (result.is_success() ? "Succeeded" : "Failed") << '\n';
        std::cout << "\tInitial guess:  " << q0.transpose() << '\n';
        std::cout << "\tSolution:       " << q_sol.transpose() << '\n';
        std::cout << "\n\tee pose goal: "
                  << ee_pose_goal.translation().transpose() << '\n';
        std::cout << "\n\tee pose sol:  "
                  << ee_pose_sol.translation().transpose() << '\n';
      }
    }
  }
}

BENCHMARK(BenchmarkRelaxedIk);

BENCHMARK_MAIN();