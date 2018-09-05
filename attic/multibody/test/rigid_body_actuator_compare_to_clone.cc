#include "drake/multibody/test/rigid_body_actuator_compare_to_clone.h"

#include <memory>

#include "drake/common/text_logging.h"
#include "drake/multibody/rigid_body.h"
#include "drake/multibody/test/rigid_body_compare_to_clone.h"

namespace drake {
namespace multibody {
namespace test {
namespace rigid_body_actuator {

bool CompareToClone(const RigidBodyActuator& original,
                    const RigidBodyActuator& clone) {
  if (original.name_ != clone.name_) {
    drake::log()->debug(
        "CompareToClone(RigidBodyActuator): Names mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.name_, clone.name_);
    return false;
  }
  if (!rigid_body::CompareToClone(*original.body_, *clone.body_)) {
    drake::log()->debug(
        "CompareToClone(RigidBodyActuator): Bodies mismatch.");
    return false;
  }
  if (original.reduction_ != clone.reduction_) {
    drake::log()->debug(
        "CompareToClone(RigidBodyActuator): Reduction mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.reduction_, clone.reduction_);
    return false;
  }
  if (original.effort_limit_min_ != clone.effort_limit_min_) {
    drake::log()->debug(
        "CompareToClone(RigidBodyActuator): Minimum effort limits mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.effort_limit_min_, clone.effort_limit_min_);
    return false;
  }
  if (original.effort_limit_max_ != clone.effort_limit_max_) {
    drake::log()->debug(
        "CompareToClone(RigidBodyActuator): Maximum effort limits mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.effort_limit_max_, clone.effort_limit_max_);
    return false;
  }
  return true;
}

}  // namespace rigid_body_actuator
}  // namespace test
}  // namespace multibody
}  // namespace drake
