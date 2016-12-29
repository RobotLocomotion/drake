#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"

#include <memory.h>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/multibody/constraint/rigid_body_constraint.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/rigid_body_tree.h"

using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::aligned_allocator;
using std::string;
using std::vector;
using std::unique_ptr;
using std::make_unique;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

void VerifyIiwaTree(const RigidBodyTree<double>& tree) {
  std::map<std::string, int> name_to_idx = tree.computePositionNameToIndexMap();

  int joint_idx = 0;
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_1"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_1"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_2"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_2"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_3"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_3"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_4"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_4"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_5"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_5"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_6"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_6"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_7"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_7"] == joint_idx++);
}

std::unique_ptr<PiecewisePolynomialTrajectory> SimpleCartesianWayPointPlanner(
    const RigidBodyTreed& tree, const std::string& link_to_constrain,
    const std::vector<Eigen::Vector3d>& way_point_list,
    const std::vector<double>& time_stamps) {
  DRAKE_DEMAND(way_point_list.size() == time_stamps.size());

  VectorXd zero_conf = tree.getZeroConfiguration();

  MatrixXd q0(tree.get_num_positions(), time_stamps.size());
  for (size_t i = 0; i < time_stamps.size(); ++i) {
    q0.col(i) = zero_conf;
  }

  vector<unique_ptr<WorldPositionConstraint>> constraint_unique_ptr_list;
  vector<RigidBodyConstraint*> constraint_ptr_list;

  vector<Eigen::Vector2d> time_window_list = TimeWindowBuilder(time_stamps);

  RigidBodyTreed* tree_ptr = const_cast<RigidBodyTreed*>(&tree);
  // Populates constraints.
  for (size_t i = 0; i < way_point_list.size(); ++i) {
    const double kCartesianPositionTolerance = 0.005;
    Vector3d pos_lb =
        way_point_list[i] - Vector3d::Constant(kCartesianPositionTolerance);
    Vector3d pos_ub =
        way_point_list[i] + Vector3d::Constant(kCartesianPositionTolerance);

    // TODO(naveenoid) : WorldPositionConstraint needs fixing to accept a
    // reference to the RigidBodyTree instead of a pointer.
    unique_ptr<WorldPositionConstraint> wpc =
        make_unique<WorldPositionConstraint>(
            tree_ptr, tree.FindBodyIndex(link_to_constrain), Vector3d::Zero(),
            pos_lb, pos_ub, time_window_list.at(i));

    // Stores the unique_ptr
    constraint_unique_ptr_list.push_back(std::move(wpc));
    constraint_ptr_list.push_back(constraint_unique_ptr_list.at(i).get());
  }

  IKoptions ikoptions(tree_ptr);
  vector<int> info(time_stamps.size(), 0);
  MatrixXd q_sol(tree.get_num_positions(), time_stamps.size());
  vector<string> infeasible_constraint;

  // TODO(naveenoid) : inverseKinPointwise needs fixing to accept a reference
  // to the RigidBodyTree instead of a pointer.
  inverseKinPointwise(tree_ptr, time_stamps.size(), time_stamps.data(), q0, q0,
                      constraint_ptr_list.size(), constraint_ptr_list.data(),
                      ikoptions, &q_sol, info.data(), &infeasible_constraint);
  bool info_good = true;
  for (size_t i = 0; i < time_stamps.size(); ++i) {
    log()->info("INFO[{}] = {} ", i, info[i]);
    if (info[i] != 1) {
      info_good = false;
    }
  }

  if (!info_good) {
    DRAKE_ABORT_MSG("inverseKinPointwise failed to compute a valid solution.");
  }

  std::vector<MatrixXd> knots(time_stamps.size());
  for (size_t i = 0; i < time_stamps.size(); ++i) {
    // We only use column 0 of the matrix in knots (for joint positions),
    // so we write a vector.
    knots[i] = q_sol.col(i);
  }

  return make_unique<PiecewisePolynomialTrajectory>(
      PiecewisePolynomial<double>::FirstOrderHold(time_stamps, knots));
}

std::vector<Eigen::Vector2d> TimeWindowBuilder(
    const std::vector<double>& time_stamps, double lower_ratio,
    double upper_ratio) {
  DRAKE_DEMAND(lower_ratio < upper_ratio);
  std::vector<Eigen::Vector2d> time_window_list;

  for (size_t i = 0; i < time_stamps.size(); ++i) {
    Eigen::Vector2d time_window;
    if (i == 0) {
      // If its the first (or only) time stamp
      time_window << 0,
          time_stamps[0] + lower_ratio * (time_stamps[1] - time_stamps[0]);
    } else if (i == time_stamps.size() - 1) {
      // If its the last time stamp
      time_window << time_stamps[i - 1] +
                         upper_ratio * (time_stamps[i] - time_stamps[i - 1]),
          time_stamps[i];
    } else {
      time_window << time_stamps[i - 1] +
                         upper_ratio * (time_stamps[i] - time_stamps[i - 1]),
          time_stamps[i] + lower_ratio * (time_stamps[i + 1] - time_stamps[i]);
    }
    time_window_list.push_back(time_window);
  }
  return time_window_list;
}

void CreateTreedFromFixedModelAtPose(const std::string& model_file_name,
                                     RigidBodyTreed* tree,
                                     const Vector3d& position,
                                     const Vector3d& orientation) {
  auto weld_to_frame = std::allocate_shared<RigidBodyFrame<double>>(
      aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr, position,
      orientation);

  // TODO(naveenoid) : consider implementing SDF version of this method.
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() + model_file_name, drake::multibody::joints::kFixed,
      weld_to_frame, tree);
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
