#include "drake/multibody/rigid_body_actuator.h"

#include <string>

#include "drake/common/text_logging.h"
#include "drake/multibody/rigid_body.h"

RigidBodyActuator::RigidBodyActuator(const std::string& name,
                                     const RigidBody<double>* body,
                                     double reduction,
                                     double effort_limit_min,
                                     double effort_limit_max)
    : name_(name),
      body_(body),
      reduction_(reduction),
      effort_limit_min_(effort_limit_min),
      effort_limit_max_(effort_limit_max) {}

bool RigidBodyActuator::CompareToClone(const RigidBodyActuator& other) const {
  if (this->name_ != other.name_) {
    drake::log()->debug(
        "RigidBodyActuator::CompareToClone(): Names mismatch:\n"
        "  - this: {}\n"
        "  - other: {}",
        this->name_, other.name_);
    return false;
  }
  if (!this->body_->CompareToClone(*other.body_)) {
    drake::log()->debug(
        "RigidBodyActuator::CompareToClone(): Bodies mismatch.");
    return false;
  }
  if (this->reduction_ != other.reduction_) {
    drake::log()->debug(
        "RigidBodyActuator::CompareToClone(): Reduction mismatch:\n"
        "  - this: {}\n"
        "  - other: {}",
        this->reduction_, other.reduction_);
    return false;
  }
  if (this->effort_limit_min_ != other.effort_limit_min_) {
    drake::log()->debug(
        "RigidBodyActuator::CompareToClone(): Minimum effort limits mismatch:\n"
        "  - this: {}\n"
        "  - other: {}",
        this->effort_limit_min_, other.effort_limit_min_);
    return false;
  }
  if (this->effort_limit_max_ != other.effort_limit_max_) {
    drake::log()->debug(
        "RigidBodyActuator::CompareToClone(): Maximum effort limits mismatch:\n"
        "  - this: {}\n"
        "  - other: {}",
        this->effort_limit_max_, other.effort_limit_max_);
    return false;
  }
  return true;
}
