#include "drake/systems/plants/model_element_id.h"

namespace drake {
namespace systems {
namespace plants {

ModelElementId::ModelElementId(const std::string& instance_name,
                               const std::string& model_name,
                               const std::string& element_name,
                               int model_instance_id)
    : instance_name_(instance_name),
      model_name_(model_name),
      element_name_(element_name),
      model_instance_id_(model_instance_id) {}

const std::string& ModelElementId::get_instance_name() const {
  return instance_name_;
}

const std::string& ModelElementId::get_model_name() const {
  return model_name_;
}

const std::string& ModelElementId::get_element_name() const {
  return element_name_;
}

int ModelElementId::get_model_instance_id() const { return model_instance_id_; }

}  // namespace plants
}  // namespace systems
}  // namespace drake
