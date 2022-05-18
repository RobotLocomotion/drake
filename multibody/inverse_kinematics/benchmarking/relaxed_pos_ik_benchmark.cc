// @file
// Benchmark for InverseKinematics.
//
// This benchmark is intended to analyze the performance of nonlinear
// optimization with position constraints.

#include "drake/common/find_resource.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/mathematical_program_result.h"
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

BENCHMARK_F(RelaxedPosIkBenchmark, Iiwa)(benchmark::State& state) {
  // Formulate an inverse kinematics problem for Kuka iiwa considering only the
  // position component with relaxation. The objective of this benchmark is
  // to run an A/B comparison for different gradient evaluations for the
  // position constraints.

  // Find the model file for Kuka iiwa.
  const std::string iiwa_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/iiwa7/"
      "iiwa7_no_collision.sdf");
  // Create a continuous-time plant.
  multibody::MultibodyPlant<double> plant(0.0);
  // Create a parser for the plant.
  multibody::Parser parser{&plant};
  //   Load the model into the parser.
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

  // Define the end-effector link.
  const std::string ee_link_name = "iiwa_link_7";
  const multibody::Body<double>& ee_body = plant.GetBodyByName(ee_link_name);
  const multibody::Frame<double>& ee_frame = plant.GetFrameByName(ee_link_name);

  // Create an IK object.
  multibody::InverseKinematics relaxed_ik(plant, true);
  solvers::MathematicalProgram* prog = relaxed_ik.get_mutable_prog();

  // Define a uniform position relaxation and the position bound variables.
  const Eigen::Vector3d pos_tol = 1e-4 * Eigen::Vector3d::Ones();

  // Get the joint position limits for random generation by leveraging the
  // fact that iiwa has symmetric position limits.
  const Eigen::VectorXd joint_pos_limits(plant.GetPositionUpperLimits());
  // Generate the random goal configurations.
  const int num_rand_goals = 10;
  std::vector<math::RigidTransformd> ee_pose_goal(num_rand_goals);
  for (int i = 0; i < num_rand_goals; ++i) {
    context->get_mutable_continuous_state()
        .get_mutable_generalized_position()
        .SetFromVector(
            Eigen::VectorXd::Random(7).cwiseProduct(joint_pos_limits));
    // Evaluate the corresponding end-effector position.
    ee_pose_goal[i] = plant.EvalBodyPoseInWorld(*context, ee_body);
  }

  // Sample a random initial guess assuming that the random range [-1, 1]
  // does not violate any of the joint position limits.
  const int num_rand_init_guess = 3;
  const Eigen::MatrixXd q0(Eigen::MatrixXd::Random(7, num_rand_init_guess));

  for (auto _ : state) {
    // Generate 10 goals randomly within joint position limits.
    for (int i = 0; i < num_rand_goals; ++i) {
      // Remove all constraints from the program.
      auto constraints = prog->GetAllConstraints();
      for (auto constraint : constraints) {
        prog->RemoveConstraint(constraint);
      }
      // Create the task in terms of position constraints on the end effector.
      relaxed_ik.AddPositionConstraint(ee_frame, Eigen::Vector3d::Zero(),
                                       plant.world_frame(),
                                       ee_pose_goal[i].translation() - pos_tol,
                                       ee_pose_goal[i].translation() + pos_tol);

      // Solve each task using three random initial guesses.
      for (int j = 0; j < num_rand_init_guess; ++j) {
        // Set the initial guess.
        prog->SetInitialGuess(relaxed_ik.q(), q0.col(j));

        // Solve the relaxed IK problem.
        solvers::MathematicalProgramResult result = solvers::Solve(*prog);
        // Get the solution.
        auto q_sol = result.GetSolution();
        // Evaluate the end-effector pose for the solution.
        context->get_mutable_continuous_state()
            .get_mutable_generalized_position()
            .SetFromVector(q_sol);
        auto ee_pose_sol = plant.EvalBodyPoseInWorld(*context, ee_body);

        // Confirm that the optimization has succeeded.
        DRAKE_DEMAND(result.is_success());
        // Check constraint satisfaction by providing a margin of 20% w.r.t. the
        // relaxation to account for the constraint tolerance of the solver.
        DRAKE_DEMAND(
            (ee_pose_sol.translation().array() <=
             ee_pose_goal[i].translation().array() + 1.2 * pos_tol.array())
                .all());
        DRAKE_DEMAND(
            (ee_pose_sol.translation().array() >=
             ee_pose_goal[i].translation().array() - 1.2 * pos_tol.array())
                .all());
      }
    }
  }
}

}  // namespace
}  // namespace inverse_kinematics
}  // namespace multibody
}  // namespace drake

BENCHMARK_MAIN();