#include "drake/multibody/test/rigid_body_compare_to_clone.h"

#include <memory>

#include "drake/common/text_logging.h"
#include "drake/multibody/rigid_body.h"

namespace drake {
namespace multibody {
namespace test {
namespace rigid_body {

bool CompareToClone(const RigidBody<double>& original,
    const RigidBody<double>& clone) {
  if (original.get_name() != clone.get_name()) {
    drake::log()->debug(
        "CompareToClone(RigidBody): name mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.get_name(),
        clone.get_name());
    return false;
  }
  if (original.get_model_name() != clone.get_model_name()) {
    drake::log()->debug(
        "CompareToClone(RigidBody): model name mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.get_model_name(),
        clone.get_model_name());
    return false;
  }
  if (original.get_model_instance_id() != clone.get_model_instance_id()) {
    drake::log()->debug(
        "CompareToClone(RigidBody): model instance ID mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.get_model_instance_id(),
        clone.get_model_instance_id());
    return false;
  }
  if (original.get_body_index() != clone.get_body_index()) {
    drake::log()->debug(
        "CompareToClone(RigidBody): body index mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.get_body_index(),
        clone.get_body_index());
    return false;
  }
  if (original.get_position_start_index() !=
      clone.get_position_start_index()) {
    drake::log()->debug(
        "CompareToClone(RigidBody): position start index mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.get_position_start_index(),
        clone.get_position_start_index());
    return false;
  }
  if (original.get_velocity_start_index() !=
      clone.get_velocity_start_index()) {
    drake::log()->debug(
        "CompareToClone(RigidBody): velocity start index mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.get_velocity_start_index(),
        clone.get_velocity_start_index());
    return false;
  }
  if (original.get_mass() != clone.get_mass()) {
    drake::log()->debug(
        "CompareToClone(RigidBody): mass mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.get_mass(),
        clone.get_mass());
    return false;
  }
  if (original.get_center_of_mass() != clone.get_center_of_mass()) {
    drake::log()->debug(
        "CompareToClone(RigidBody): center of mass mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.get_center_of_mass(),
        clone.get_center_of_mass());
    return false;
  }
  if (original.get_spatial_inertia() != clone.get_spatial_inertia()) {
    drake::log()->debug(
        "CompareToClone(RigidBody): spatial inertia mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.get_spatial_inertia(),
        clone.get_spatial_inertia());
    return false;
  }
  return true;
}

}  // namespace rigid_body
}  // namespace test
}  // namespace multibody
}  // namespace drake
