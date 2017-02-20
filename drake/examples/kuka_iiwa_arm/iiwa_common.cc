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

void SetPositionControlledIiwaGains(Eigen::VectorXd* Kp,
                                    Eigen::VectorXd* Ki,
                                    Eigen::VectorXd* Kd) {
  // TODO(naveenoid) : Update the gains pending careful system identification
  // of the real KUKA iiwa arm's (hidden) low level control.

  // These values for the position gains Kp were chosen from a
  // combination of intuition based on the inertias / masses of the
  // links and some trial and error to achieve reasonably quick
  // critically damped position control when used along with the
  // gravity compensator.
  Kp->resize(7);
  *Kp << 100, 100, 100, 20, 10, 20, 1;
  Kd->resize(Kp->size());
  for (int i = 0; i < Kp->size(); i++) {
    // Derivative gains are computed as the square-root of the corresponding
    // position gains as a reasonable approximation of critically damped
    // behaviour.
    (*Kd)[i] = std::sqrt((*Kp)[i]);
  }
  *Ki = Eigen::VectorXd::Zero(7);
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
