#pragma once

#include <map>
#include <string>
#include <unordered_map>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/parsing/frame_cache.h"
#include "drake/multibody/parsing/sdf_link.h"
#include "drake/multibody/parsing/sdf_joint.h"

namespace drake {
namespace multibody {
namespace parsing {
namespace sdf {

/// This class provides a representation of a `<model>` entry within an SDF
/// file.
class SDFModel {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SDFModel)

  /// Creates a new SDF model object specification with the given `model_name`.
  /// Per SDF specification, `model_name` must not match any other model name in
  /// the world (<world>).
  explicit SDFModel(const std::string& model_name) :
      name_(model_name), frame_cache_(model_name) {}

  /// Returns the name of `this` link.
  const std::string& name() const { return name_; }

  int get_num_links() const {
    return static_cast<int>(links_.size());
  }

  int get_num_joints() const {
    return static_cast<int>(joints_.size());
  }

  SDFLink& AddLink(const std::string& link_name) {
    const int link_index = get_num_links();
    links_.emplace_back(link_name);
    links_name_to_index_map_.insert({link_name, link_index});
    return links_.back();
  }
  
  SDFJoint& AddJoint(const std::string& joint_name,
                     const std::string& parent_link_name,
                     const std::string& child_link_name,
                     const std::string& joint_type) {
    const int joint_index = get_num_joints();
    joints_.emplace_back(
        joint_name, parent_link_name, child_link_name, joint_type);
    joints_name_to_index_map_.insert({joint_name, joint_index});
    return joints_.back();
  }

  const std::vector<SDFLink>& get_links() const {
    return links_;
  }

  const std::vector<SDFJoint>& get_joints() const {
    return joints_;
  }

  const SDFLink& GetLinkByName(const std::string& link_name) const {
    const auto it = links_name_to_index_map_.find(link_name);
    if (it == links_name_to_index_map_.end()) {
      throw std::runtime_error(
          "Link \"" + link_name + "\" not found in model \"" + this->name()
              + "\"");
    }
    return links_[it->second];
  }

  const SDFJoint& GetJointByName(const std::string& joint_name) const {
    const auto it = joints_name_to_index_map_.find(joint_name);
    if (it == joints_name_to_index_map_.end()) {
      throw std::runtime_error(
          "Joint \"" + joint_name + "\" not found in model \"" + this->name()
              + "\"");
    }
    return joints_[it->second];
  }

  /// Caches the pose `X_MF` of a frame F named `frame_name` in a "measured-in"
  /// frame M named `masured_in_frame_name`.
  void CachePose(const std::string& measured_in_frame_name,
                 const std::string& frame_name,
                 const Isometry3<double>& X_MF) {
    frame_cache_.Update(measured_in_frame_name, frame_name, X_MF);
  }

  /// Returns the pose `X_MF` of a frame F named `frame_name` in a "measured-in"
  /// frame M named `masured_in_frame_name`.
  Isometry3<double> GetPose(const std::string& measured_in_frame_name,
                            const std::string& frame_name) const {
    return frame_cache_.Transform(measured_in_frame_name, frame_name);
  }

 private:
  SDFLink& GetMutableLinkByName(const std::string& link_name) {
    const auto it = links_name_to_index_map_.find(link_name);
    if (it == links_name_to_index_map_.end()) {
      throw std::runtime_error(
          "Link \"" + link_name + "\" not found in model \"" + this->name()
              + "\"");
    }
    return links_[it->second];
  }

  // Name of the root frame of this cache.
  std::string name_;

  // Vector of links in the model.
  std::vector<SDFLink> links_;
  
  // Mapping from link name to an index into std::vector links_.
  std::unordered_map<std::string, int> links_name_to_index_map_;

  // Vector of joints in the model.
  std::vector<SDFJoint> joints_;
  
  // Mapping from joint name to an index into std::vector joints_.
  std::unordered_map<std::string, int> joints_name_to_index_map_;

  // Model "remembers" frames defined during parsing and their relative poses.
  FrameCache<double> frame_cache_;
};

}  // namespace sdf
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
