#include "drake/multibody/parsing/model_directives.h"

#include "drake/common/text_logging.h"

namespace drake {
namespace multibody {
namespace parsing {

bool AddWeld::IsValid() const {
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

bool AddModel::IsValid() const {
  if (file.empty()) {
    drake::log()->error("add_model: `file` must be non-empty");
    return false;
  } else if (name.empty()) {
    drake::log()->error("add_model: `name` must be non-empty");
    return false;
  }
  for (const auto& [body_name, pose] : default_free_body_pose) {
    if (!pose.IsDeterministic()) {
      drake::log()->error(
          "add_model: `default_free_body_pose` must specify a "
          "deterministic transform, not a distribution.");
      return false;
    }
  }
  return true;
}

bool AddModelInstance::IsValid() const {
  if (name.empty()) {
    drake::log()->error("add_model_instance: `name` must be non-empty");
    return false;
  }
  return true;
}

bool AddFrame::IsValid() const {
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

bool AddCollisionFilterGroup::IsValid() const {
  if (name.empty()) {
    drake::log()->error("add_collision_filter_group: `name` must be non-empty");
    return false;
  } else if (members.empty() && member_groups.empty()) {
    drake::log()->error(
        "add_collision_filter_group:"
        " at least one of `members` or `member_groups` must be non-empty");
    return false;
  }
  return true;
}

bool AddDirectives::IsValid() const {
  if (file.empty()) {
    drake::log()->error("add_directives: `file` must be non-empty");
    return false;
  }
  return true;
}

bool ModelDirective::IsValid() const {
  const bool unique = (add_model.has_value() + add_model_instance.has_value() +
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

bool ModelDirectives::IsValid() const {
  for (auto& directive : directives) {
    if (!directive.IsValid()) return false;
  }
  return true;
}

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
