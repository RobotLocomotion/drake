#include "drake/examples/kuka_iiwa_arm/iiwa_ik_planner.h"

#include <iostream>
#include <list>
#include <memory>

#include "drake/multibody/constraint/rigid_body_constraint.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_ik.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

IiwaIkPlanner::IiwaIkPlanner(const std::string& model_path,
                             const std::string& end_effector_link_name,
                             const Isometry3<double>& base_to_world, int seed)
    : rand_generator_(seed) {
  auto base_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr,
      base_to_world);

  robot_ = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFile(
      model_path, multibody::joints::kFixed, base_frame, robot_.get());

  default_initial_pos_tolerance_ << 0.1, 0.1, 0.1;
  default_initial_rot_tolerance_ = 0.1;

  SetEndEffector(end_effector_link_name);
}

IiwaIkPlanner::IiwaIkPlanner(const std::string& model_path,
                             const std::string& end_effector_link_name,
                             std::shared_ptr<RigidBodyFrame<double>> base,
                             int seed)
    : rand_generator_(seed) {
  robot_ = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFile(
      model_path, multibody::joints::kFixed, base, robot_.get());

  default_initial_pos_tolerance_ << 0.1, 0.1, 0.1;
  default_initial_rot_tolerance_ = 0.1;

  SetEndEffector(end_effector_link_name);
}

std::unique_ptr<PiecewisePolynomialTrajectory>
IiwaIkPlanner::GenerateFirstOrderHoldTrajectoryFromCartesianWaypoints(
    const std::vector<double>& time_stamps,
    const std::vector<Vector3<double>>& way_point_list,
    const Vector3<double>& position_tol) {
  DRAKE_DEMAND(way_point_list.size() == time_stamps.size());
  std::vector<IkCartesianWaypoint> waypoints(time_stamps.size());
  for (int i = 0; i < static_cast<int>(time_stamps.size()); ++i) {
    waypoints[i].time = time_stamps[i];
    waypoints[i].pose.translation() = way_point_list[i];
    waypoints[i].pos_tol = position_tol;
    waypoints[i].rot_tol = 1;
  }

  IkResult ik_res;
  PlanSequentialTrajectory(waypoints, robot_->getZeroConfiguration(), &ik_res);
  return GenerateFirstOrderHoldTrajectory(ik_res);
}

bool IiwaIkPlanner::PlanSequentialTrajectory(
    const std::vector<IkCartesianWaypoint>& waypoints,
    const VectorX<double>& q_current, IkResult* ik_res) {
  DRAKE_DEMAND(ik_res);
  int num_dof = robot_->get_num_positions();
  int num_steps = static_cast<int>(waypoints.size());

  VectorX<double> q_prev = q_current;
  VectorX<double> q0 = q_current;
  VectorX<double> q_sol = q_current;

  ik_res->time.resize(num_steps + 1);
  ik_res->info.resize(num_steps + 1);
  ik_res->q.resize(num_dof, num_steps + 1);
  ik_res->time[0] = 0;
  ik_res->info[0] = 1;
  ik_res->q.col(0) = q_current;

  int ctr = 0;

  const int kRelaxPosTol = 0;
  const int kRelaxRotTol = 1;

  for (const auto& waypoint : waypoints) {
    // Sets the initial constraints guess bigger than the desired tolerance.
    Vector3<double> pos_tol = default_initial_pos_tolerance_;
    double rot_tol = default_initial_rot_tolerance_;

    if (!waypoint.enforce_quat) rot_tol = 0;

    // Sets mode to reduce position tolerance.
    int mode = kRelaxPosTol;

    int random_ctr = 0;
    int relaxed_ctr = 0;

    // Solves point IK with constraint fiddling and random start.
    while (true) {
      if (!waypoint.enforce_quat) DRAKE_DEMAND(mode == kRelaxPosTol);

      bool res = SolveIk(waypoint, q0, q_prev, pos_tol, rot_tol, &q_sol);

      if (res) {
        // Alternates between kRelaxPosTol and kRelaxRotTol
        if (mode == kRelaxPosTol && waypoint.enforce_quat) {
          rot_tol /= 2.;
          mode = kRelaxRotTol;
        } else {
          pos_tol /= 2.;
          mode = kRelaxPosTol;
        }
        // Sets the initial guess to the current solution.
        q0 = q_sol;

        // Breaks if the current tolerance is below given threshold.
        if ((rot_tol <= waypoint.rot_tol) &&
            (pos_tol.array() <= waypoint.pos_tol.array()).all()) {
          break;
        }
      } else {
        // Relaxes the constraints no solution is found.
        if (mode == kRelaxRotTol && waypoint.enforce_quat) {
          rot_tol *= 1.5;
        } else {
          pos_tol *= 1.5;
        }
        relaxed_ctr++;
      }

      // Switches to a different initial guess and start over if we have relaxed
      // the constraints for max times.
      if (relaxed_ctr > 10) {
        // Make a random initial guess.
        q0 = robot_->getRandomConfiguration(rand_generator_);
        // Resets constraints tolerance.
        pos_tol = default_initial_pos_tolerance_;
        rot_tol = default_initial_rot_tolerance_;
        if (!waypoint.enforce_quat) rot_tol = 0;
        mode = kRelaxPosTol;

        std::cout << "FAILED at max constraint relaxing iter: " << relaxed_ctr
                  << "\n";
        relaxed_ctr = 0;
        random_ctr++;
      }

      // Admits failure and returns false.
      if (random_ctr > 50) {
        std::cout << "FAILED at max random retries: " << random_ctr << "\n";
        return false;
      }
    }

    // Sets next IK's initial and bias to current solution.
    q_prev = q_sol;
    q0 = q_sol;

    ik_res->time[ctr + 1] = waypoint.time;
    ik_res->info[ctr + 1] = 1;

    ik_res->q.col(ctr + 1) = q_sol;
    ctr++;
  }

  return true;
}

bool IiwaIkPlanner::SolveIk(const IkCartesianWaypoint& waypoint,
                            const VectorX<double>& q0,
                            const VectorX<double>& q_nom,
                            const Vector3<double>& pos_tol, double rot_tol,
                            VectorX<double>* q_res) {
  DRAKE_DEMAND(q_res);
  std::vector<RigidBodyConstraint*> constraint_array;
  std::vector<int> info(1);
  std::vector<std::string> infeasible_constraint;
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
                               math::rotmat2quat(waypoint.pose.linear()),
                               rot_tol, Vector2<double>::Zero());
  if (waypoint.enforce_quat) {
    constraint_array.push_back(&quat_con);
  }

  inverseKin(robot_.get(), q0, q_nom, constraint_array.size(),
             constraint_array.data(), ikoptions, q_res, info.data(),
             &infeasible_constraint);

  if (info[0] != 1) {
    return false;
  }

  return true;
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
