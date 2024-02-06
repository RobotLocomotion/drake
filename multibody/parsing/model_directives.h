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
#include "drake/common/text_logging.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace multibody {
namespace parsing {

/// Directive to add a weld between two named frames, a parent and a child.
struct AddWeld {
  bool IsValid() const {
    if (parent.empty()) {
      drake::log()->error("add_weld: `parent` must be non-empty");
      return false;
    } else if (child.empty()) {
      drake::log()->error("add_weld: `child` must be non-empty");
      return false;
    }
    if (X_PC) {
      if (X_PC->base_frame) {
        drake::log()->error(
            "add_weld: `X_PC` must not specify a `base_frame`; the pose is "
            "always in the parent frame.");
        return false;
      }
      if (!X_PC->IsDeterministic()) {
        drake::log()->error(
            "add_weld: `X_PC` must specify a deterministic transform, not a "
            "distribution.");
        return false;
      }
    }
    return true;
  }

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
  bool IsValid() const {
    if (file.empty()) {
      drake::log()->error("add_model: `file` must be non-empty");
      return false;
    } else if (name.empty()) {
      drake::log()->error("add_model: `name` must be non-empty");
      return false;
    }
    for (const auto& [body_name, pose] : default_free_body_pose) {
      if (pose.base_frame) {
        drake::log()->error(
            "add_model: `default_free_body_pose` must not specify a "
            "`base_frame`; the pose is always in the world frame.");
        return false;
      }
      if (!pose.IsDeterministic()) {
        drake::log()->error(
            "add_model: `default_free_body_pose` must specify a "
            "deterministic transform, not a distribution.");
        return false;
      }
    }
    return true;
  }

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
  /// Map of joint_name => default position vector.
  std::map<std::string, Eigen::VectorXd> default_joint_positions;
  /// Map of body_name => default free body pose.
  std::map<std::string, drake::schema::Transform> default_free_body_pose;
};

/// Directive to add an empty, named model instance to a scene.
struct AddModelInstance {
  bool IsValid() const {
    if (name.empty()) {
      drake::log()->error("add_model_instance: `name` must be non-empty");
      return false;
    }
    return true;
  }

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
  bool IsValid() const {
    if (name.empty()) {
      drake::log()->error("add_frame: `name` must be non-empty");
      return false;
    } else if (!X_PF.base_frame || X_PF.base_frame->empty()) {
      drake::log()->error("add_frame: `X_PF.base_frame` must be defined");
      return false;
    } else if (!X_PF.IsDeterministic()) {
      drake::log()->error(
          "add_frame: `X_PF` must specify a deterministic transform, not a "
          "distribution.");
      return false;
    }
    return true;
  }

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
  bool IsValid() const {
    if (name.empty()) {
      drake::log()->error(
          "add_collision_filter_group: `name` must be non-empty");
      return false;
    } else if (members.empty() && member_groups.empty()) {
      drake::log()->error(
          "add_collision_filter_group:"
          " at least one of `members` or `member_groups` must be non-empty");
      return false;
    }
    return true;
  }

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
  bool IsValid() const {
    if (file.empty()) {
      drake::log()->error("add_directives: `file` must be non-empty");
      return false;
    }
    return true;
  }

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
  bool IsValid() const {
    const bool unique =
        (add_model.has_value() + add_model_instance.has_value() +
         add_frame.has_value() + add_weld.has_value() +
         add_collision_filter_group.has_value() +
         add_directives.has_value()) == 1;
    if (!unique) {
      drake::log()->error(
          "directive: Specify one of `add_model`, `add_model_instance`, "
          "`add_frame`, `add_collision_filter_group`, or `add_directives`");
      return false;
    } else if (add_model) {
      return add_model->IsValid();
    } else if (add_model_instance) {
      return add_model_instance->IsValid();
    } else if (add_frame) {
      return add_frame->IsValid();
    } else if (add_weld) {
      return add_weld->IsValid();
    } else if (add_collision_filter_group) {
      return add_collision_filter_group->IsValid();
    } else {
      return add_directives->IsValid();
    }
  }

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
  bool IsValid() const {
    for (auto& directive : directives) {
      if (!directive.IsValid()) return false;
    }
    return true;
  }

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(directives));
  }

  std::vector<ModelDirective> directives;
};

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
