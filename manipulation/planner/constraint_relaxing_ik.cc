#include "drake/manipulation/planner/constraint_relaxing_ik.h"

#include <memory>
#include <stdexcept>

#include "drake/common/text_logging.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace manipulation {
namespace planner {
namespace {
constexpr int kDefaultRandomSeed = 1234;
}  // namespace

ConstraintRelaxingIk::ConstraintRelaxingIk(
    const std::string& model_path,
    const std::string& end_effector_link_name)
    : rand_generator_(kDefaultRandomSeed),
      plant_(0) {
  const auto model_instance =
      multibody::Parser(&plant_).AddModelFromFile(model_path);

  // Check if our robot is welded to the world.  If not, try welding the first
  // link.
  if (plant_.GetBodiesWeldedTo(plant_.world_body()).size() <= 1) {
    const std::vector<multibody::BodyIndex> bodies =
        plant_.GetBodyIndices(model_instance);
    plant_.WeldFrames(plant_.world_frame(),
                      plant_.get_body(bodies[0]).body_frame());
  }
  plant_.Finalize();

  SetEndEffector(end_effector_link_name);
}

bool ConstraintRelaxingIk::PlanSequentialTrajectory(
    const std::vector<IkCartesianWaypoint>& waypoints,
    const VectorX<double>& q_current,
    std::vector<Eigen::VectorXd>* q_sol_out) {
  DRAKE_DEMAND(q_sol_out != nullptr);
  int num_steps = static_cast<int>(waypoints.size());

  VectorX<double> q0 = q_current;
  VectorX<double> q_sol = q_current;

  q_sol_out->clear();
  q_sol_out->push_back(q_current);

  int step_ctr = 0;
  int relaxed_ctr = 0;
  int random_ctr = 0;

  enum class RelaxMode { kRelaxPosTol = 0, kRelaxRotTol = 1 };

  // These numbers are arbitrarily picked by siyuan.
  const int kMaxNumInitialGuess = 50;
  const int kMaxNumConstraintRelax = 10;
  const Vector3<double> kInitialPosTolerance(0.01, 0.01, 0.01);
  const double kInitialRotTolerance = 0.01;
  const double kConstraintShrinkFactor = 0.5;
  const double kConstraintGrowFactor = 1.5;

  for (const auto& waypoint : waypoints) {
    // Sets the initial constraints guess bigger than the desired tolerance.
    Vector3<double> pos_tol = kInitialPosTolerance;
    double rot_tol = kInitialRotTolerance;

    if (!waypoint.constrain_orientation) rot_tol = 0;

    // Sets mode to reduce position tolerance.
    RelaxMode mode = RelaxMode::kRelaxPosTol;

    // Solves point IK with constraint fiddling and random start.
    while (true) {
      if (!waypoint.constrain_orientation)
        DRAKE_DEMAND(mode == RelaxMode::kRelaxPosTol);

      std::vector<int> info;
      std::vector<std::string> infeasible_constraints;
      bool res = SolveIk(waypoint, q0, pos_tol, rot_tol, &q_sol);

      if (res) {
        // Breaks if the current tolerance is below given threshold.
        if ((rot_tol <= waypoint.rot_tol) &&
            (pos_tol.array() <= waypoint.pos_tol.array()).all()) {
          break;
        }

        // Alternates between kRelaxPosTol and kRelaxRotTol
        if (mode == RelaxMode::kRelaxPosTol && waypoint.constrain_orientation) {
          rot_tol *= kConstraintShrinkFactor;
          mode = RelaxMode::kRelaxRotTol;
        } else {
          pos_tol *= kConstraintShrinkFactor;
          mode = RelaxMode::kRelaxPosTol;
        }
        // Sets the initial guess to the current solution.
        q0 = q_sol;
      } else {
        // Relaxes the constraints no solution is found.
        if (mode == RelaxMode::kRelaxRotTol && waypoint.constrain_orientation) {
          rot_tol *= kConstraintGrowFactor;
        } else {
          pos_tol *= kConstraintGrowFactor;
        }
        relaxed_ctr++;
      }

      // Switches to a different initial guess and start over if we have relaxed
      // the constraints for max times.
      if (relaxed_ctr > kMaxNumConstraintRelax) {
        // Make a random initial guess.
        std::unique_ptr<systems::Context<double>> context =
            plant_.CreateDefaultContext();
        plant_.SetRandomState(*context, &context->get_mutable_state(),
                              &rand_generator_);
        q0 = plant_.GetPositions(*context);
        // Resets constraints tolerance.
        pos_tol = kInitialPosTolerance;
        rot_tol = kInitialRotTolerance;
        if (!waypoint.constrain_orientation) rot_tol = 0;
        mode = RelaxMode::kRelaxPosTol;
        drake::log()->warn("IK FAILED at max constraint relaxing iter: " +
                           std::to_string(relaxed_ctr));
        relaxed_ctr = 0;
        random_ctr++;
      }

      // Admits failure and returns false.
      if (random_ctr > kMaxNumInitialGuess) {
        drake::log()->error("IK FAILED at max random starts: " +
                            std::to_string(random_ctr));
        // Returns information about failure.
        q_sol_out->push_back(q_sol);
        return false;
      }
    }

    // Sets next IK's initial and bias to current solution.
    q0 = q_sol;
    q_sol_out->push_back(q_sol);
    step_ctr++;
  }

  DRAKE_DEMAND(static_cast<int>(q_sol_out->size()) == num_steps + 1);
  return true;
}

bool ConstraintRelaxingIk::SolveIk(
    const IkCartesianWaypoint& waypoint,
    const VectorX<double>& q0,
    const Vector3<double>& pos_tol, double rot_tol,
    VectorX<double>* q_res) {
  DRAKE_DEMAND(q_res != nullptr);

  multibody::InverseKinematics ik(plant_);

  // Adds a position constraint.
  Vector3<double> pos_lb = waypoint.pose.translation() - pos_tol;
  Vector3<double> pos_ub = waypoint.pose.translation() + pos_tol;

  ik.AddPositionConstraint(
      plant_.get_body(end_effector_body_idx_).body_frame(),
      Vector3<double>::Zero(),
      plant_.world_frame(), pos_lb, pos_ub);

  if (waypoint.constrain_orientation) {
    ik.AddOrientationConstraint(
        plant_.world_frame(), waypoint.pose.rotation(),
        plant_.get_body(end_effector_body_idx_).body_frame(),
        math::RotationMatrixd(), rot_tol);
  }

  const auto result = solvers::Solve(ik.prog(), q0);
  if (!result.is_success()) {
    return false;
  }

  *q_res = result.get_x_val();
  return true;
}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
