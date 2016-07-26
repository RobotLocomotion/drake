#include "drake/systems/plants/element_id.h"

namespace drake {
namespace systems {
namespace plants {

ElementId::ElementId(const std::string& instance_name,
                     const std::string& model_name,
                     const std::string& element_name, int model_id)
    : instance_name_(instance_name),
      model_name_(model_name),
      element_name_(element_name),
      model_id_(model_id) {}

const std::string& ElementId::get_instance_name() const {
  return instance_name_;
}

const std::string& ElementId::get_model_name() const { return model_name_; }

const std::string& ElementId::get_element_name() const { return element_name_; }

int ElementId::get_model_id() const { return model_id_; }

}  // namespace plants
}  // namespace systems
}  // namespace drake
