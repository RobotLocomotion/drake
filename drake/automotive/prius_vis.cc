#include "drake/automotive/prius_vis.h"

#include <Eigen/Geometry>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/kinematics_cache.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/rigid_body_plant/create_load_robot_message.h"

using std::unique_ptr;
using std::vector;

namespace drake {

using multibody::joints::kRollPitchYaw;
using systems::rendering::PoseBundle;

namespace automotive {
template <typename T> constexpr double PriusVis<T>::kVisOffset;

template <typename T>
PriusVis<T>::PriusVis(int id, const std::string& name)
    : CarVis<T>(id, name),
      tree_(new RigidBodyTree<T>()) {
  const std::string kSdfFilename =
      GetDrakePath() + "/automotive/models/prius/prius_with_lidar.sdf";
  parsers::sdf::AddModelInstancesFromSdfFileToWorld(
      kSdfFilename, kRollPitchYaw, tree_.get());
  lcmt_viewer_load_robot load_message =
      multibody::CreateLoadRobotMessage<T>(*tree_);
  for (const auto& link : load_message.link) {
    if (link.name != RigidBodyTreeConstants::kWorldName) {
      vis_elements_.push_back(link);
      vis_elements_.back().robot_num = id;
    }
  }
}

template <typename T>
const vector<lcmt_viewer_link_data>& PriusVis<T>::GetVisElements() const {
  return vis_elements_;
}

template <typename T>
systems::rendering::PoseBundle<T> PriusVis<T>::CalcPoses(
    const Isometry3<T>& X_WM) const {
  // Computes X_MV, the transform from the visualization's frame to the model's
  // frame. The 'V' in the variable name stands for "visualization". This is
  // necessary because the model frame's origin is centered at the midpoint of
  // the vehicle's rear axle whereas the visualization frame's origin is
  // centered in the middle of a body called "chassis_floor". The axes of the
  // two frames are parallel with each other. However, the distance between the
  // origins of the two frames is 1.40948 m along the model's x-axis.
  const Isometry3<T> X_MV(Eigen::Translation<T, 3>(T(1.40948) /* x offset */,
                                                   T(0)       /* y offset */,
                                                   T(0)       /* z offset */));
  const Isometry3<T> X_WV = X_WM * X_MV;
  const auto rotation = X_WV.linear();
  const auto transform = X_WV.translation();
  Vector3<T> rpy = rotation.eulerAngles(2, 1, 0);
  VectorX<T> q = VectorX<T>::Zero(tree_->get_num_positions());
  q(0) = transform.x();
  q(1) = transform.y();
  q(2) = transform.z();
  q(3) = rpy(2);
  q(4) = rpy(1);
  q(5) = rpy(0);
  const KinematicsCache<T> cache = tree_->doKinematics(q);

  PoseBundle<T> result(this->num_poses());
  int result_index{0};
  for (int i = 0; i < tree_->get_num_bodies(); ++i) {
    if (i != RigidBodyTreeConstants::kWorldBodyIndex) {
      const RigidBody<T>& body = tree_->get_body(i);
      const drake::Isometry3<T> X_WB =
          tree_->CalcBodyPoseInWorldFrame(cache, body);
      result.set_pose(result_index, X_WB);
      result.set_name(result_index, body.get_name());
      result.set_model_instance_id(result_index, this->id());
      ++result_index;
    }
  }
  DRAKE_ASSERT(result_index == this->num_poses());
  return result;
}

// These instantiations must match the API documentation in
// prius_vis.h.
template class PriusVis<double>;

}  // namespace automotive
}  // namespace drake
