#pragma once
/// @file
/// Defines the YAML schema for the model directives language, which is used
/// to assemble multiple SDF and URDF files in a single MultibodyPlant.
///
/// For more information on how structures are converted to and from YAML,
/// refer to @ref yaml_serialization "YAML Serialization".
///
/// See `multibody/parsing/README_model_directives.md` for more info.

#include <map>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/common/name_value.h"
#include "drake/common/schema/transform.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace multibody {
namespace parsing {

/// Directive to add a weld between two named frames, a parent and a child.
struct AddWeld {
  bool IsValid() const;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(parent));
    a->Visit(DRAKE_NVP(child));
    a->Visit(DRAKE_NVP(X_PC));
  }

  /// Parent frame. Can specify scope.
  std::string parent;
  /// Child frame. Can (and should) specify scope.
  std::string child;
  /// Relative transform between the parent frame P and the child frame C. If
  /// unspecified, the Identity transform will be used.
  std::optional<drake::schema::Transform> X_PC{};
};

/// Directive to add a model from a URDF or SDFormat file to a scene, using a
/// given name for the added instance.
struct AddModel {
  bool IsValid() const;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(file));
    a->Visit(DRAKE_NVP(name));
    a->Visit(DRAKE_NVP(default_joint_positions));
    a->Visit(DRAKE_NVP(default_free_body_pose));
  }

  /// The `package://` URI of the file to add.
  std::string file;
  /// The model instance name.
  std::string name;
  /// Map of joint_name => default position vector. Each joint name must be
  /// a name within the scope of the model added by this directive. The name
  /// must not contains *this* model's scoped name (nor that of any previously
  /// added model).
  std::map<std::string, Eigen::VectorXd> default_joint_positions;
  /// Map of body_name or frame_name => default free body pose. The name must
  /// be a name within the scope of the model added by this directive. The name
  /// must not be scoped (i.e., no "foo::link", just "link"). If the name is
  /// empty, then the posed frame will be the body frame of the model's sole
  /// body (and if the model has >1 body then it is an error).
  ///
  /// For deformable bodies, the body pose defines the world pose of the
  /// reference configuration, i.e. the undeformed geometry of the deformable
  /// body will have this pose in the world frame.
  ///
  /// However, the schema::Transform associated with that named body/frame can
  /// define a `base_frame` referring to any frame that has been added prior to
  /// or including this declaration.  The named frame must *always* be a scoped
  /// name, even if its part of the model added by this directive.
  ///
  /// @warning there are two important implications for the named frame if the
  /// transform's `base_frame` is not the world (explicitly or implicitly by
  /// omission):
  ///
  ///  1. The named body will *not* be considered a "floating base" body (see
  ///     @ref mbp_working_with_free_bodies "Working with free bodies"). Calls
  ///     to MultibodyPlant::SetDefaultFreeBodyPose() will have no effect on an
  ///     allocated context. If you want to change its default pose after
  ///     adding the model, you need to acquire the body's joint and set the
  ///     new default pose on the joint directly. Note: what you will
  ///     *actually* be posing is the *named* frame. If it's the name of the
  ///     body, you will be posing the body. If it's a frame affixed to the
  ///     body frame, you will be posing the fixed frame (with the body offset
  ///     based on the relationship between the two frames).
  ///  2. The body associated with the named frame will have a six-dof joint
  ///     between itself and the body associated with the transform's
  ///     `base_frame`. When interpreting the qs for the "named" body, it is the
  ///     six-dof pose of the body measured and expressed in the parent frame
  ///     (transform's `base_frame`). This is true whether setting the position
  ///     values in the resulting joint directly or using the
  ///     @ref mbp_working_with_free_bodies "MultibodyPlant free body APIs".
  ///  3. If the body is deformable, non-world `base_frame` is an error.
  ///
  /// @warning There should not already be a joint in the model between the two
  /// bodies implied by the named frames.
  std::map<std::string, drake::schema::Transform> default_free_body_pose;
};

/// Directive to add an empty, named model instance to a scene.
struct AddModelInstance {
  bool IsValid() const;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(name));
  }

  /// The model instance name.
  std::string name;
};

/// Directive to add a Frame to the scene.  The added frame must have a name
/// and a transform with a base frame and offset.
struct AddFrame {
  bool IsValid() const;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(name));
    a->Visit(DRAKE_NVP(X_PF));
  }

  /// Name of frame to be added. If scope is specified, will override model
  /// instance; otherwise, will use `X_PF.base_frame`s instance.
  std::string name;
  /// Pose of frame to be added, `F`, w.r.t. parent frame `P` (as defined by
  /// `X_PF.base_frame`).
  drake::schema::Transform X_PF;
};

/// Directive to add a collision filter group.  This directive is analogous to
/// @ref tag_drake_collision_filter_group in XML model formats.
struct AddCollisionFilterGroup {
  bool IsValid() const;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(name));
    a->Visit(DRAKE_NVP(members));
    a->Visit(DRAKE_NVP(member_groups));
    a->Visit(DRAKE_NVP(model_namespace));
    a->Visit(DRAKE_NVP(ignored_collision_filter_groups));
  }

  /// Name of group to be added.  This is an unscoped name, and must be
  /// unique either globally or within its specified model namespace.
  std::string name;
  /// Optional model namespace.  Allows `name` to be reused between models
  /// and lets you use the scoped name in `ignored_collision_filter_groups`.
  std::optional<std::string> model_namespace;
  /// Names of members of the group. May be scoped and refer to bodies of
  /// already added models. This data is analogous to a sequence of
  /// @ref tag_drake_member in XML model formats.
  std::vector<std::string> members;
  /// Names of groups to add en masse as members of the group. May be scoped
  /// and refer to bodies of already added models. This data is analogous to a
  /// sequence of @ref tag_drake_member_group in XML model formats.
  std::vector<std::string> member_groups;
  /// Names of groups against which to ignore collisions. If another group is
  /// named, collisions between this group and that group will be ignored. If
  /// this group is named, collisions within this group will be ignored. Names
  /// may be scoped and refer to other groups defined elsewhere in this file or
  /// transitively included directives or model files. This data is analogous
  /// to a sequence of @ref tag_drake_ignored_collision_filter_group in XML
  /// model formats.
  std::vector<std::string> ignored_collision_filter_groups;
};

/// Directive to incorporate another model directives file, optionally with
/// its elements prefixed with a namespace.
struct AddDirectives {
  bool IsValid() const;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(file));
    a->Visit(DRAKE_NVP(model_namespace));
  }

  /// The `package://` URI of the file to add.
  std::string file;
  /// Namespaces base model instance for processing directive files.
  /// Affects scoping (i.e. the following members):
  /// - AddModel::name
  /// - AddModelInstance::name
  /// - AddFrame::name
  /// - AddWeld::parent
  /// - AddWeld::child
  /// - AddFrame::X_PF::base_frame
  /// - AddCollisionFilterGroup::name
  /// - AddCollisionFilterGroup::members
  /// - AddCollisionFilterGroup::ignored_colllision_filter_groups
  /// - AddDirectives::model_namespace
  /// See `README.md` for example references and namespacing.
  std::optional<std::string> model_namespace;
};

// TODO(eric.cousineau): Change this (and relevant YAML files) to use tags.
/// Union structure for model directives.
///
/// @note This was designed before support for `std::variant<>` was around,
/// and thus we used a parent field, rather than a YAML tag, to designate the
/// intended type for the directive.
struct ModelDirective {
  bool IsValid() const;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(add_model));
    a->Visit(DRAKE_NVP(add_model_instance));
    a->Visit(DRAKE_NVP(add_frame));
    a->Visit(DRAKE_NVP(add_weld));
    a->Visit(DRAKE_NVP(add_collision_filter_group));
    a->Visit(DRAKE_NVP(add_directives));
  }

  std::optional<AddModel> add_model;
  std::optional<AddModelInstance> add_model_instance;
  std::optional<AddFrame> add_frame;
  std::optional<AddWeld> add_weld;
  std::optional<AddCollisionFilterGroup> add_collision_filter_group;
  std::optional<AddDirectives> add_directives;
};

/// Top-level structure for a model directives yaml file schema.
struct ModelDirectives {
  bool IsValid() const;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(directives));
  }

  std::vector<ModelDirective> directives;
};

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
