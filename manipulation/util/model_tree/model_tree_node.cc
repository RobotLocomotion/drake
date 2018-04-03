#include "drake/manipulation/util/model_tree/model_tree_node.h"

#include <set>

#include "drake/common/text_logging.h"

using drake::nullopt;
using drake::optional;
using drake::multibody::joints::FloatingBaseType;

namespace drake {
namespace manipulation {
namespace util {
namespace model_tree {

bool operator==(const PredecessorInfo& info_0, const PredecessorInfo& info_1) {
  drake::log()->trace(
      "info_0.model_instance_name == "
      "info_1.model_instance_name: "
      "{}\n"
      "        info_0.model_instance_name: {}\n"
      "        info_1.model_instance_name: {}",
      info_0.model_instance_name == info_1.model_instance_name,
      info_0.model_instance_name, info_1.model_instance_name);
  drake::log()->trace(
      "info_0.body_or_frame_name == "
      "info_1.body_or_frame_name: "
      "{}\n"
      "        info_0.body_or_frame_name: {}\n"
      "        info_1.body_or_frame_name: {}",
      info_0.body_or_frame_name == info_1.body_or_frame_name,
      info_0.body_or_frame_name, info_1.body_or_frame_name);
  return (info_0.model_instance_name == info_1.model_instance_name) &&
         (info_0.body_or_frame_name == info_1.body_or_frame_name) &&
         (info_0.is_frame == info_1.is_frame);
}

bool operator==(const ModelFile& file_0, const ModelFile& file_1) {
  drake::log()->trace(
      "file_0.absolute_path == file_1.absolute_path: {}\n"
      "        file_0.absolute_path: {}\n"
      "        file_1.absolute_path: {}",
      file_0.absolute_path == file_1.absolute_path, file_0.absolute_path,
      file_1.absolute_path);
  drake::log()->trace(
      "file_0.type == file_1.type: {}\n"
      "        file_0.type: {}\n"
      "        file_1.type: {}",
      file_0.type == file_1.type, static_cast<int>(file_0.type),
      static_cast<int>(file_1.type));
  return (file_0.absolute_path == file_1.absolute_path) &&
         (file_0.type == file_1.type);
}

bool ModelTreeNode::operator==(const ModelTreeNode& other) const {
  drake::log()->trace(
      "name_ == other.name_: {}\n"
      "        name:       {}\n"
      "        other.name: {}",
      name_ == other.name_, name_, other.name_);
  drake::log()->trace(
      "parent_name_ == other.parent_name_: {}\n"
      "        parent_name:       {}\n"
      "        other.parent_name: {}",
      parent_name_ == other.parent_name_, parent_name_, other.parent_name_);
  drake::log()->trace("model_file_ == other.model_file_: {}",
                      model_file_ == other.model_file_);
  drake::log()->trace("predecessor_info_ == other.predecessor_info_: {}",
                      predecessor_info_ == other.predecessor_info_);
  drake::log()->trace(
      "parent_predecessor_info_ == other.parent_predecessor_info_: {}",
      parent_predecessor_info_ == other.parent_predecessor_info_);
  drake::log()->trace(
      "X_PB.IsNearlyEqualTo(info_1.X_PB, 0.0): {}\n"
      "X_PB:\n"
      "{}\n"
      "other.X_PB:\n"
      "{}\n",
      X_PB().IsNearlyEqualTo(other.X_PB(), 0.0), X_PB().GetAsMatrix4(),
      other.X_PB().GetAsMatrix4());
  drake::log()->trace("base_joint_type == other.base_joint_type_: {}",
                      base_joint_type_ == other.base_joint_type_);
  return (name_ == other.name_) && (model_file_ == other.model_file_) &&
         (parent_name_ == other.parent_name_) &&
         (predecessor_info_ == other.predecessor_info_) &&
         (parent_predecessor_info_ == other.parent_predecessor_info_) &&
         X_PB_.IsNearlyEqualTo(other.X_PB_, 0.0) &&
         (base_joint_type_ == other.base_joint_type_) &&
         (children_ == other.children_);
}

ModelTreeNode::ModelTreeNode(
    const std::string& node_name, const drake::optional<ModelFile>& model_file,
    const drake::optional<PredecessorInfo>& node_predecessor_info,
    const drake::math::Transform<double>& X_PB,
    FloatingBaseType base_joint_type,
    const std::vector<ModelTreeNode>& children)
    : name_(node_name),
      model_file_(model_file),
      predecessor_info_(node_predecessor_info),
      X_PB_(X_PB),
      base_joint_type_(base_joint_type),
      children_(children) {
  std::set<std::string> child_names;
  for (auto& child : children_) {
    if (!child_names.insert(child.name_).second) {
      throw std::runtime_error("Duplicate child node name: " + child.name_);
    }
  }
  UpdateChildren();
}

bool ModelTreeNode::is_predecessor_parent_base_frame() const {
  return !predecessor_info_;
}

std::string ModelTreeNode::ParentNamePrefix() const {
  if (parent_name_.empty()) {
    return "";
  }
  return parent_name_ + "/";
}

std::string ModelTreeNode::name() const { return ParentNamePrefix() + name_; }

optional<PredecessorInfo> ModelTreeNode::predecessor_info() const {
  if (is_predecessor_parent_base_frame()) {
    return parent_predecessor_info_;
  }
  return PredecessorInfo(
      ParentNamePrefix() + predecessor_info_->model_instance_name,
      predecessor_info_->body_or_frame_name, predecessor_info_->is_frame);
}

drake::math::Transform<double> ModelTreeNode::X_PB() const {
  if (is_predecessor_parent_base_frame()) {
    return parent_X_PB_ * X_PB_;
  }
  return X_PB_;
}

FloatingBaseType ModelTreeNode::base_joint_type() const {
  if (is_predecessor_parent_base_frame() &&
      base_joint_type_ == FloatingBaseType::kFixed) {
    return parent_base_joint_type_;
  }
  return base_joint_type_;
}

void ModelTreeNode::UpdateChildren() {
  for (ModelTreeNode& child : children_) {
    child.parent_name_ = name();
    child.parent_predecessor_info_ = predecessor_info();
    child.parent_X_PB_ = X_PB();
    child.parent_base_joint_type_ = base_joint_type();
    child.UpdateChildren();
  }
}

}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
