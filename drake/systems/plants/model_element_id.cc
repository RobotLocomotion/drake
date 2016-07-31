#include "drake/systems/plants/model_element_id.h"

namespace drake {
namespace systems {
namespace plants {

ModelElementId::ModelElementId(const std::string& model_instance_name,
                               const std::string& model_name,
                               const std::string& element_name,
                               int model_instance_id)
    : model_instance_name_(model_instance_name),
      model_name_(model_name),
      element_name_(element_name),
      model_instance_id_(model_instance_id) {}

const std::string& ModelElementId::get_model_instance_name() const {
  return model_instance_name_;
}

const std::string& ModelElementId::get_model_name() const {
  return model_name_;
}

const std::string& ModelElementId::get_element_name() const {
  return element_name_;
}

int ModelElementId::get_model_instance_id() const { return model_instance_id_; }

// TODO(liang.fok) Remove this method. See:
// https://github.com/RobotLocomotion/drake/issues/2990
void ModelElementId::set_model_instance_name(const std::string& model_name) {
  model_name_ = model_name;
}

// TODO(liang.fok) Remove this method. See:
// https://github.com/RobotLocomotion/drake/issues/2990
void ModelElementId::set_model_name(const std::string& model_name) {
  model_name_ = model_name;
}

// TODO(liang.fok) Remove this method. See:
// https://github.com/RobotLocomotion/drake/issues/2990
void ModelElementId::set_element_name(const std::string& element_name) {
  element_name_ = element_name;
}

// TODO(liang.fok) Remove this method. See:
// https://github.com/RobotLocomotion/drake/issues/2990
void ModelElementId::set_model_instance_id(int model_instance_id) {
  model_instance_id_ = model_instance_id;
}

bool operator==(const ModelElementId& left_element,
                const ModelElementId& right_element) {
  return left_element.get_model_instance_name() ==
             right_element.get_model_instance_name() &&
         left_element.get_model_name() == right_element.get_model_name() &&
         left_element.get_element_name() == right_element.get_element_name() &&
         left_element.get_model_instance_id() ==
             right_element.get_model_instance_id();
}

bool operator!=(const ModelElementId& left_element,
                const ModelElementId& right_element) {
  return !(left_element == right_element);
}

}  // namespace plants
}  // namespace systems
}  // namespace drake
