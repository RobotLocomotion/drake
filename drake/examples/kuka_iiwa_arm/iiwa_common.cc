#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"

#include <map>
#include <string>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/multibody/constraint/rigid_body_constraint.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/parser_urdf.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/rigid_body_tree.h"

using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
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

unique_ptr<PiecewisePolynomialTrajectory> PositionViaPointCartesianPlanner(
    Vector3d robot_base_position, Vector3d robot_base_orientation,
    string robot_urdf_file, vector<Vector3d> object_position_list,
    vector<double> time_stamps) {
  DRAKE_DEMAND(object_position_list.size() == time_stamps.size());
  RigidBodyTree<double> tree{};

  auto weld_to_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr,
      robot_base_position, robot_base_orientation);

  parsers::urdf::AddModelInstanceFromUrdfFile(GetDrakePath() + robot_urdf_file,
                                              multibody::joints::kFixed,
                                              weld_to_frame, &tree);

  VectorXd zero_conf = tree.getZeroConfiguration();

  VectorXd joint_lb = zero_conf - VectorXd::Constant(7, 0.01);
  VectorXd joint_ub = zero_conf + VectorXd::Constant(7, 0.01);

  MatrixXd q0(tree.get_num_positions(), time_stamps.size());
  for (size_t i = 0; i < time_stamps.size(); ++i) {
    q0.col(i) = zero_conf;
  }

  vector<unique_ptr<WorldPositionConstraint>> constraint_unique_ptr_list;
  vector<RigidBodyConstraint*> constraint_ptr_list;

  // Populates constraints.
  for (size_t ctr = 0; ctr < object_position_list.size(); ++ctr) {
    // The following bit of logic creates time windows for which the constraint
    // is to be active from the given time stamps. The logic ensures that the
    // windows never overlap for a monotonically increasing time stamp vector.
    // For time stamp t(k), the window is located at
    // [t(k-1) + 0.4 * ( t(k) - t(k-1)), t(k) + 0.5 * (t(k+1) - t(k))]
    // i.e. between 40% of the previous time step to 50% of the subsequent time
    // step, with the boundary conditions [0, t(f) + 0.5*(t(f) - t(f-1))].
    Vector2d time_window;
    if (ctr == 0) {
      // If its the first (or only) time stamp
      time_window << 0,
          time_stamps[0] + 0.4 * (time_stamps[1] - time_stamps[0]);
    } else if (ctr == object_position_list.size() - 1) {
      // If its the last time stamp
      time_window << time_stamps[ctr - 1] +
                         0.5 * (time_stamps[ctr] - time_stamps[ctr - 1]),
          time_stamps[ctr] + 0.4 * (time_stamps[ctr] - time_stamps[ctr - 1]);
    } else {
      time_window << time_stamps[ctr - 1] +
                         0.5 * (time_stamps[ctr] - time_stamps[ctr - 1]),
          time_stamps[ctr] + 0.4 * (time_stamps[ctr + 1] - time_stamps[ctr]);
    }

    const double kCartesianPositionTolerance = 0.005;
    Vector3d pos_lb = object_position_list[ctr] -
                      Vector3d::Constant(kCartesianPositionTolerance);
    Vector3d pos_ub = object_position_list[ctr] +
                      Vector3d::Constant(kCartesianPositionTolerance);

    unique_ptr<WorldPositionConstraint> wpc =
        make_unique<WorldPositionConstraint>(
            &tree, tree.FindBodyIndex("iiwa_link_ee"), Vector3d::Zero(), pos_lb,
            pos_ub, time_window);

    // Stores the unique_ptr.
    constraint_unique_ptr_list.push_back(std::move(wpc));
    constraint_ptr_list.push_back(constraint_unique_ptr_list.at(ctr).get());
  }

  IKoptions ikoptions(&tree);
  vector<int> info(time_stamps.size(), 0);
  MatrixXd q_sol(tree.get_num_positions(), time_stamps.size());
  vector<string> infeasible_constraint;

  inverseKinPointwise(&tree, time_stamps.size(), time_stamps.data(), q0, q0,
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

  return make_unique<PiecewisePolynomialTrajectory>(q_sol, time_stamps);
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
