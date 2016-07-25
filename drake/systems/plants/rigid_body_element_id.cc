#include "drake/systems/plants/rigid_body_element_id.h"

namespace drake {
namespace systems {
namespace plants {

RigidBodyElementId::RigidBodyElementId(const std::string& instance_name,
                                       const std::string& model_name,
                                       const std::string& element_name)
    : instance_name_(instance_name),
      model_name_(model_name),
      element_name_(element_name) {}

const std::string& RigidBodyElementId::get_instance_name() const {
  return instance_name_;
}

const std::string& RigidBodyElementId::get_model_name() const {
  return model_name_;
}

const std::string& RigidBodyElementId::get_element_name() const {
  return element_name_;
}

}  // namespace plants
}  // namespace systems
}  // namespace drake
