#pragma once

#include <map>
#include <string>
#include <unordered_map>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/parsing/sdf/sdf_link.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace parsing {

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

  SDFLink& AddLink(const std::string& link_name) {
    const int link_index = get_num_links();
    links_.emplace_back(link_name);
    links_name_to_index_map_.insert({link_name, link_index});
    return links_.back();
  }

  const std::vector<SDFLink>& get_links() const {
    return links_;
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

  void SetLinkPoseInModelFrame(const std::string& link_name,
                               const Isometry3<double>& X_DL) {
    SDFLink& link = GetMutableLinkByName(link_name);
    link.set_pose_in_model(X_DL);
    frame_cache_.Update(this->name(), link_name, X_DL);
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

  // Mapping from link name to an index into std::vector links_.
  std::unordered_map<std::string, int> links_name_to_index_map_;

  // Vector of links in the model.
  std::vector<SDFLink> links_;

  // Cached poses. Used to compute relative poses between frames during parsing.
  FrameCache<double> frame_cache_;
};

}  // namespace parsing
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
