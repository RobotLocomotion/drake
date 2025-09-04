// @file
// Benchmark for InverseKinematics.
//
// This benchmark is intended to analyze the performance of nonlinear
// optimization with position constraints.

#include <memory>
#include <string>
#include <vector>

#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/inverse_kinematics/position_constraint.h"
#include "drake/multibody/parsing/parser.h"
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

  // Define constants.
  // A uniform relaxation for end-effector position.
  const Eigen::Vector3d kPosTol = 1e-3 * Eigen::Vector3d::Ones();
  // The number of random goals.
  const int kNumRandGoals = 10;
  // The number of random initial guesses for each goal.
  const int kNumRandInitGuess = 2;

  // Find the model file for Kuka iiwa.
  const std::string iiwa_url =
      "package://drake_models/iiwa_description/sdf/iiwa7_no_collision.sdf";
  // Create a continuous-time plant.
  multibody::MultibodyPlant<double> plant(0.0);
  // Create a parser for the plant.
  multibody::Parser parser{&plant};
  // Load the model into the parser.
  parser.AddModelsFromUrl(iiwa_url);
  // Attach the base of the robot into the world frame.
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("iiwa_link_0"));
  // Finalize the plant.
  plant.Finalize();
  // Create a context.
  std::unique_ptr<systems::Context<double>> context =
      plant.CreateDefaultContext();

  // Define the end-effector link and get the corresponding body and frame.
  const std::string ee_link_name = "iiwa_link_7";
  const multibody::RigidBody<double>& ee_body =
      plant.GetBodyByName(ee_link_name);
  const multibody::Frame<double>& ee_frame = plant.GetFrameByName(ee_link_name);

  // Create an IK object and get its mathetical program.
  multibody::InverseKinematics relaxed_ik(plant, true);
  solvers::MathematicalProgram* prog = relaxed_ik.get_mutable_prog();

  // Generate random goal configurations.
  std::vector<math::RigidTransformd> ee_pose_goal(kNumRandGoals);
  for (int i = 0; i < kNumRandGoals; ++i) {
    // Sample a random joint pose assuming that the random range [-1, 1] rad
    // does not violate any of the joint position limits.
    context->get_mutable_continuous_state()
        .get_mutable_generalized_position()
        .SetFromVector(Eigen::VectorXd::Random(plant.num_positions()));
    // Evaluate the corresponding end-effector pose.
    ee_pose_goal[i] = plant.EvalBodyPoseInWorld(*context, ee_body);
  }

  // Sample random initial guesses assuming that the random range [-1, 1] rad
  // does not violate any of the joint position limits.
  const Eigen::MatrixXd q0(
      Eigen::MatrixXd::Random(plant.num_positions(), kNumRandInitGuess));

  // Create the task in terms of position constraints on the end effector.
  auto pos_constraint = std::make_shared<PositionConstraint>(
      &plant, plant.world_frame(), -kPosTol, kPosTol, ee_frame,
      Eigen::Vector3d::Zero(), context.get());
  prog->AddConstraint(pos_constraint, relaxed_ik.q());

  for (auto _ : state) {
    // Solve the problem for each goal and initial guess.
    for (int i = 0; i < kNumRandGoals; ++i) {
      // Update the task constraint.
      pos_constraint->set_bounds(ee_pose_goal[i].translation() - kPosTol,
                                 ee_pose_goal[i].translation() + kPosTol);

      for (int j = 0; j < kNumRandInitGuess; ++j) {
        // Set the initial guess.
        prog->SetInitialGuess(relaxed_ik.q(), q0.col(j));

        // Solve the problem. The success of optimization is not checked as
        // we're rather interested in measuring the speed of constraint
        // evaluation.
        solvers::Solve(*prog);
      }
    }
  }
}

}  // namespace
}  // namespace inverse_kinematics
}  // namespace multibody
}  // namespace drake
