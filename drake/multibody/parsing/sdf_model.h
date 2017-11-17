#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/parsing/frame_cache.h"
#include "drake/multibody/parsing/sdf_joint.h"
#include "drake/multibody/parsing/sdf_link.h"

namespace drake {
namespace multibody {
namespace parsing {

/// This class provides a representation of a `<model>` element within a given
/// SDF specification.
/// For details on the specification of models, including conventions and
/// default values, please refer to the documentation for the
/// <a href="http://sdformat.org/spec?ver=1.6&elem=model">
/// &lt;model&gt; element</a>.
class SdfModel {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SdfModel)

  /// Creates a new SDF model object specification with the given `model_name`.
  /// Per SDF specification, `model_name` must not match any other model name in
  /// the world (`<world>`).
  explicit SdfModel(const std::string& model_name) :
      name_(model_name), frame_cache_(model_name) {}

  /// Returns the name of `this` model.
  const std::string& name() const { return name_; }

  /// Returns the number of links (corresponding to `<link>` elements) in
  /// `this` model.
  int get_num_links() const {
    return static_cast<int>(links_.size());
  }

  /// Returns the number of joints (corresponding to `<joint>` elements) in
  /// `this` model.
  int get_num_joints() const {
    return static_cast<int>(joints_.size());
  }


  /// @name Methods to add model components.
  /// @{
  // TODO(amcastro-tri): have a method VerifyModel(), or similar, which can
  // verify the correctness of some invariants. For instance, joints should make
  // reference to links that do exist in this model.

  /// Adds a new link named `link_name` to `this` model and returns a reference
  /// to the newly added link.
  /// This method aborts if `this` model already contains a link named
  /// `link_name`.
  SdfLink& AddLink(const std::string& link_name) {
    DRAKE_DEMAND(!HasLink(link_name));
    const int link_index = get_num_links();
    links_.emplace_back(link_name);
    link_name_to_index_map_.insert({link_name, link_index});
    return links_.back();
  }

  /// Adds a new joint named `joint_name` to `this` model and returns a
  /// reference to the newly added joint. Please refer to the SdfJoint class's
  /// documentation for details on the arguments for this method.
  /// This method aborts if `this` model already contains a joint named
  /// `joint_name`.
  SdfJoint& AddJoint(const std::string& joint_name,
                     const std::string& parent_link_name,
                     const std::string& child_link_name,
                     const std::string& joint_type) {
    DRAKE_DEMAND(!HasJoint(joint_name));
    const int joint_index = get_num_joints();
    joints_.emplace_back(
        joint_name, parent_link_name, child_link_name, joint_type);
    joint_name_to_index_map_.insert({joint_name, joint_index});
    return joints_.back();
  }
  /// @}

  /// Returns an std::vector of SdfLink objects containing all the links in
  /// `this` model.
  const std::vector<SdfLink>& get_links() const {
    return links_;
  }

  /// Returns an std::vector of SdfJoint objects containing all the joints in
  /// `this` model.
  const std::vector<SdfJoint>& get_joints() const {
    return joints_;
  }

  /// Returns `true` if `this` model contains a link named `link_name`.
  bool HasLink(const std::string& link_name) const {
    return link_name_to_index_map_.count(link_name) > 0;
  }

  /// Returns `true` if `this` model contains a joint named `joint_name`.
  bool HasJoint(const std::string& joint_name) const {
    return joint_name_to_index_map_.count(joint_name) > 0;
  }

  /// Returns a const reference to the SdfLink object with unique name within
  /// this model `link_name`.
  /// This method throws a std::runtime_error if the model does not contain a
  /// link named `link_name`.
  const SdfLink& GetLinkByName(const std::string& link_name) const {
    const auto it = link_name_to_index_map_.find(link_name);
    if (it == link_name_to_index_map_.end()) {
      throw std::runtime_error(
          "Link '" + link_name + "' not found in model '" + this->name() + "'");
    }
    return links_[it->second];
  }

  /// Returns a const reference to the SdfJoint object with unique name within
  /// this model `joint_name`.
  /// This method throws a std::runtime_error if the model does not contain a
  /// joint named `joint_name`.
  const SdfJoint& GetJointByName(const std::string& joint_name) const {
    const auto it = joint_name_to_index_map_.find(joint_name);
    if (it == joint_name_to_index_map_.end()) {
      throw std::runtime_error(
          "Joint '" + joint_name + "' not found in model '" + this->name()
              + "'");
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

  /// Returns the pose `X_DF` of a frame F named `frame_name` as measured in
  /// `this` model frame D.
  Isometry3<double> GetPoseInModelFrame(const std::string& frame_name) const {
    return frame_cache_.Transform(name_, frame_name);
  }

 private:
  // The name of this model.
  std::string name_;

  // Vector of links in the model.
  std::vector<SdfLink> links_;

  // Mapping from link name to an index into std::vector links_.
  std::unordered_map<std::string, int> link_name_to_index_map_;

  // Vector of joints in the model.
  std::vector<SdfJoint> joints_;

  // Mapping from joint name to an index into std::vector joints_.
  std::unordered_map<std::string, int> joint_name_to_index_map_;

  // Model "remembers" frames defined during parsing and their relative poses.
  FrameCache<double> frame_cache_;
};

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
