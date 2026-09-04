#include "drake/multibody/parsing/model_directives.h"

#include "drake/common/text_logging.h"

namespace drake {
namespace multibody {
namespace parsing {

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;

namespace {

/* Preserves the historical IsValid() behavior: log errors and return false
without throwing. */
DiagnosticPolicy MakeLoggingPolicy() {
  DiagnosticPolicy policy;
  policy.SetActionForErrors([](const DiagnosticDetail& detail) {
    drake::log()->error(detail.message);
  });
  return policy;
}

}  // namespace

bool AddWeld::IsValid() const {
  return IsValid(MakeLoggingPolicy());
}

bool AddWeld::IsValid(const DiagnosticPolicy& diagnostic) const {
  if (parent.empty()) {
    diagnostic.Error("add_weld: `parent` must be non-empty");
    return false;
  } else if (child.empty()) {
    diagnostic.Error("add_weld: `child` must be non-empty");
    return false;
  }
  if (X_PC) {
    if (X_PC->base_frame) {
      diagnostic.Error(
          "add_weld: `X_PC` must not specify a `base_frame`; the pose is "
          "always in the parent frame.");
      return false;
    }
    if (!X_PC->IsDeterministic()) {
      diagnostic.Error(
          "add_weld: `X_PC` must specify a deterministic transform, not a "
          "distribution.");
      return false;
    }
  }
  return true;
}

bool AddModel::IsValid() const {
  return IsValid(MakeLoggingPolicy());
}

bool AddModel::IsValid(const DiagnosticPolicy& diagnostic) const {
  if (file.empty()) {
    diagnostic.Error("add_model: `file` must be non-empty");
    return false;
  } else if (name.empty()) {
    diagnostic.Error("add_model: `name` must be non-empty");
    return false;
  }
  for (const auto& [body_name, pose] : default_free_body_pose) {
    if (!pose.IsDeterministic()) {
      diagnostic.Error(
          "add_model: `default_free_body_pose` must specify a "
          "deterministic transform, not a distribution.");
      return false;
    }
  }
  return true;
}

bool AddModelInstance::IsValid() const {
  return IsValid(MakeLoggingPolicy());
}

bool AddModelInstance::IsValid(const DiagnosticPolicy& diagnostic) const {
  if (name.empty()) {
    diagnostic.Error("add_model_instance: `name` must be non-empty");
    return false;
  }
  return true;
}

bool AddFrame::IsValid() const {
  return IsValid(MakeLoggingPolicy());
}

bool AddFrame::IsValid(const DiagnosticPolicy& diagnostic) const {
  if (name.empty()) {
    diagnostic.Error("add_frame: `name` must be non-empty");
    return false;
  } else if (!X_PF.base_frame || X_PF.base_frame->empty()) {
    diagnostic.Error("add_frame: `X_PF.base_frame` must be defined");
    return false;
  } else if (!X_PF.IsDeterministic()) {
    diagnostic.Error(
        "add_frame: `X_PF` must specify a deterministic transform, not a "
        "distribution.");
    return false;
  }
  return true;
}

bool AddCollisionFilterGroup::IsValid() const {
  return IsValid(MakeLoggingPolicy());
}

bool AddCollisionFilterGroup::IsValid(
    const DiagnosticPolicy& diagnostic) const {
  if (name.empty()) {
    diagnostic.Error("add_collision_filter_group: `name` must be non-empty");
    return false;
  } else if (members.empty() && member_groups.empty()) {
    diagnostic.Error(
        "add_collision_filter_group:"
        " at least one of `members` or `member_groups` must be non-empty");
    return false;
  }
  return true;
}

bool AddDirectives::IsValid() const {
  return IsValid(MakeLoggingPolicy());
}

bool AddDirectives::IsValid(const DiagnosticPolicy& diagnostic) const {
  if (file.empty()) {
    diagnostic.Error("add_directives: `file` must be non-empty");
    return false;
  }
  return true;
}

bool ModelDirective::IsValid() const {
  return IsValid(MakeLoggingPolicy());
}

bool ModelDirective::IsValid(const DiagnosticPolicy& diagnostic) const {
  const bool unique = (add_model.has_value() + add_model_instance.has_value() +
                       add_frame.has_value() + add_weld.has_value() +
                       add_collision_filter_group.has_value() +
                       add_directives.has_value()) == 1;
  if (!unique) {
    diagnostic.Error(
        "directive: Specify one of `add_model`, `add_model_instance`, "
        "`add_frame`, `add_collision_filter_group`, or `add_directives`");
    return false;
  } else if (add_model) {
    return add_model->IsValid(diagnostic);
  } else if (add_model_instance) {
    return add_model_instance->IsValid(diagnostic);
  } else if (add_frame) {
    return add_frame->IsValid(diagnostic);
  } else if (add_weld) {
    return add_weld->IsValid(diagnostic);
  } else if (add_collision_filter_group) {
    return add_collision_filter_group->IsValid(diagnostic);
  } else {
    return add_directives->IsValid(diagnostic);
  }
}

bool ModelDirectives::IsValid() const {
  return IsValid(MakeLoggingPolicy());
}

bool ModelDirectives::IsValid(const DiagnosticPolicy& diagnostic) const {
  for (auto& directive : directives) {
    if (!directive.IsValid(diagnostic)) return false;
  }
  return true;
}

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
