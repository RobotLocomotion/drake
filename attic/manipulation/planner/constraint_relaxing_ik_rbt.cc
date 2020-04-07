#include "drake/manipulation/planner/constraint_relaxing_ik_rbt.h"

#include <memory>

#include "drake/common/text_logging.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_constraint.h"
#include "drake/multibody/rigid_body_ik.h"

namespace drake {
namespace manipulation {
namespace planner {
namespace {
constexpr int kDefaultRandomSeed = 1234;
}  // namespace

ConstraintRelaxingIkRbt::ConstraintRelaxingIkRbt(
    const std::string& model_path,
    const std::string& end_effector_link_name,
    const Isometry3<double>& base_to_world)
    : rand_generator_(kDefaultRandomSeed) {
  auto base_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr,
      base_to_world);

  robot_ = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFile(
      model_path, multibody::joints::kFixed, base_frame, robot_.get());

  SetEndEffector(end_effector_link_name);
}

bool ConstraintRelaxingIkRbt::PlanSequentialTrajectory(
    const std::vector<IkCartesianWaypoint>& waypoints,
    const VectorX<double>& q_current, IKResults* ik_res) {
  DRAKE_DEMAND(ik_res);
  int num_steps = static_cast<int>(waypoints.size());

  VectorX<double> q_prev = q_current;
  VectorX<double> q0 = q_current;
  VectorX<double> q_sol = q_current;

  ik_res->infeasible_constraints.clear();
  ik_res->info.resize(num_steps + 1);
  ik_res->q_sol.resize(num_steps + 1);
  ik_res->info[0] = 1;
  ik_res->q_sol[0] = q_current;

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
      bool res = SolveIk(waypoint, q0, q_prev, pos_tol, rot_tol, &q_sol, &info,
                         &infeasible_constraints);

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
        q0 = robot_->getRandomConfiguration(rand_generator_);
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
        ik_res->info[step_ctr + 1] = info[0];
        ik_res->q_sol[step_ctr + 1] = q_sol;
        ik_res->infeasible_constraints = infeasible_constraints;
        return false;
      }
    }

    // Sets next IK's initial and bias to current solution.
    q_prev = q_sol;
    q0 = q_sol;

    ik_res->info[step_ctr + 1] = 1;
    ik_res->q_sol[step_ctr + 1] = q_sol;
    step_ctr++;
  }

  return true;
}

bool ConstraintRelaxingIkRbt::SolveIk(
    const IkCartesianWaypoint& waypoint,
    const VectorX<double>& q0,
    const VectorX<double>& q_nom,
    const Vector3<double>& pos_tol, double rot_tol,
    VectorX<double>* q_res, std::vector<int>* info,
    std::vector<std::string>* infeasible_constraints) {
  DRAKE_DEMAND(q_res);
  DRAKE_DEMAND(info);
  DRAKE_DEMAND(infeasible_constraints);

  info->resize(1);
  std::vector<RigidBodyConstraint*> constraint_array;
  IKoptions ikoptions(robot_.get());
  ikoptions.setDebug(true);

  // Adds a position constraint.
  Vector3<double> pos_lb = waypoint.pose.translation() - pos_tol;
  Vector3<double> pos_ub = waypoint.pose.translation() + pos_tol;

  WorldPositionConstraint pos_con(robot_.get(), end_effector_body_idx_,
                                  Vector3<double>::Zero(), pos_lb, pos_ub,
                                  Vector2<double>::Zero());

  constraint_array.push_back(&pos_con);

  // Adds a rotation constraint.
  WorldQuatConstraint quat_con(robot_.get(), end_effector_body_idx_,
    math::RotationMatrix<double>::ToQuaternionAsVector4(waypoint.pose.linear()),
    rot_tol, Vector2<double>::Zero());
  if (waypoint.constrain_orientation) {
    constraint_array.push_back(&quat_con);
  }

  inverseKin(robot_.get(), q0, q_nom, constraint_array.size(),
             constraint_array.data(), ikoptions, q_res, info->data(),
             infeasible_constraints);

  return (*info)[0] == 1;
}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
