#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/math/transform.h"
#include "drake/multibody/joints/floating_base_types.h"

namespace drake {
namespace manipulation {
namespace util {
namespace model_tree {

/** Describes how a model tree node is attached to the tree.
 */
// TODO(avalenzu): Add a FindBodyOrFrame() method to RigidBodyTree and get rid
// of this nonsense.
struct PredecessorInfo {
  PredecessorInfo(const std::string& model_instance_name_in,
                  const std::string& body_or_frame_name_in,
                  bool is_frame_in)
      : model_instance_name(model_instance_name_in),
        body_or_frame_name(body_or_frame_name_in),
        is_frame(is_frame_in) {}
  std::string model_instance_name{};
  std::string body_or_frame_name{};
  bool is_frame{false};
};

/** Enumerates supported model file types.
 */
enum class ModelFileType { kUrdf, kSdf };

/** Describes a model file
 */
struct ModelFile {
  ModelFile(const std::string& absolute_path_in, ModelFileType type_in)
      : absolute_path(absolute_path_in), type(type_in) {}
  std::string absolute_path{};
  ModelFileType type{};
};

bool operator==(const PredecessorInfo& info_0, const PredecessorInfo& info_1);

bool operator==(const ModelFile& file_0, const ModelFile& file_1);

/** Represents a node in a model tree.
 */
class ModelTreeNode {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ModelTreeNode);

  /** Constructs a ModelTreeNode with the given parameters.
   * @param node_name Identifying name of this node.
   * @param model_file Describes the model file specified by this node. Pass
   *        drake::nullopt if the node does not specify a model file.
   * @param node_predecessor_info Describes the predecessor of this node (the
   *        body or frame to which this node's base is attached). Pass
   *        drake::nullopt to indicate that this node's base is attached to the
   *        base of this node's parent node. Otherwise, pass a PredecessorInfo
   *        instance describing a body or frame in one of this node's siblings.
   * @param X_PB Pose of this node's Base frame relative to its Predecessor.
   * @param base_joint_type Joint type of the joint between this node's base and
   *        its predecessor.
   * @param children Nodes to be copied and added as children of this node. Each
   *        node in `children` must have a unique `name()`.
   * @throws std::runtime_error if any elements of `children` have the same
   *         name.
   */
  ModelTreeNode(const std::string& node_name,
                const drake::optional<ModelFile>& model_file,
                const drake::optional<PredecessorInfo>& node_predecessor_info,
                const drake::math::Transform<double>& X_PB,
                drake::multibody::joints::FloatingBaseType base_joint_type,
                const std::vector<ModelTreeNode>& children);

  /** Fully qualified name of this node. For child nodes, the format of this
   * name is: `root_name/.../parent_name/node_name`
   */
  std::string name() const;

  /** Information on the model file specified by this node. Returns
   * drake::nullopt if this node does not specify a model file.
   */
  drake::optional<ModelFile> model_file() const { return model_file_; }

  /** Fully qualified information on the predecessor of this node's base frame.
   * Returns drake::nullopt if the predecessor is the world frame.
   *
   * Note that for child nodes, this will differ from the
   * `node_predecessor_info` passed to the constructor. If drake::nullopt was
   * passed to the constructor, this method will return the result of calling
   * predecessor_info() on the node's parent. If a PredecessorInfo instance
   * was passed to the constructor, the fully qualified name of the parent will
   * be prepended to the `model_instance_name`.  */
  drake::optional<PredecessorInfo> predecessor_info() const;

  /** Pose of this node's Base frame relative to its Predecessor.
   */
  drake::math::Transform<double> X_PB() const;

  /** Joint type for the joint connecting the base frame to its predecessor.
   *
   * Note that for child nodes, this may differ from the `base_joint_type`
   * passed to the constructor. If drake::nullopt and FloatingBaseType::kFixed
   * were passed to the constructor for `node_predecessor_info` and
   * `base_joint_type` respectively, then this method will return the result of
   * calling base_joint_type() on this node's parent.
   */
  drake::multibody::joints::FloatingBaseType base_joint_type() const;

  /** Children of this node.
   */
  const std::vector<ModelTreeNode>& children() const { return children_; }

  bool operator==(const ModelTreeNode& other) const;

  bool operator!=(const ModelTreeNode& other) const {
    return !operator==(other);
  }

 private:
  std::string ParentNamePrefix() const;
  void UpdateChildren();
  bool is_predecessor_parent_base_frame() const;

  std::string name_{};
  drake::optional<ModelFile> model_file_{};
  // Identifying information for the predecessor frame.
  drake::optional<PredecessorInfo> predecessor_info_{};
  // Pose of Base frame relative to Predecessor.
  drake::math::Transform<double> X_PB_{};
  // Joint type to be used to connect the base frame to the predecessor frame.
  drake::multibody::joints::FloatingBaseType base_joint_type_{};
  std::vector<ModelTreeNode> children_{};
  // Values copied from the parent of this node (if any) during a call to the
  // parent's UpdateChildren() method.
  std::string parent_name_{};
  drake::optional<PredecessorInfo> parent_predecessor_info_{};
  drake::math::Transform<double> parent_X_PB_{};
  drake::multibody::joints::FloatingBaseType parent_base_joint_type_{};
};

typedef ModelTreeNode ModelTree;

}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
