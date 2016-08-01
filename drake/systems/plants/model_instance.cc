#include "drake/systems/plants/model_instance.h"

namespace drake {
namespace systems {
namespace plants {

using std::move;
using std::unique_ptr;

ModelInstance::ModelInstance() : ModelInstance("[undefined instance name]") {
}

ModelInstance::ModelInstance(const std::string& model_instance_name) :
    model_instance_name_(model_instance_name) {
}

void ModelInstance::set_model_name(const std::string& model_name) {
  model_name_ = model_name;
}

void ModelInstance::add_joint(unique_ptr<DrakeJoint> joint) {
  joints_.push_back(move(joint));
}

void ModelInstance::add_body(unique_ptr<RigidBody> body) {
  bodies_.push_back(move(body));
}

void ModelInstance::add_frame(unique_ptr<RigidBodyFrame> frame) {
  frames_.push_back(move(frame));
}

void ModelInstance::add_loop(unique_ptr<RigidBodyLoop> loop) {
  loops_.push_back(move(loop));
}

void ModelInstance::add_actuator(unique_ptr<RigidBodyActuator> actuator) {
  actuators_.push_back(move(actuator));
}

void ModelInstance::add_sensor(unique_ptr<RigidBodySensor> sensor) {
  sensors_.push_back(move(sensor));
}

void ModelInstance::add_model_instance(ModelInstance* model_instance) {
  model_instances_.push_back(model_instance);
}

const std::string& ModelInstance::get_model_intance_name() const {
  return model_instance_name_;
}

const std::string& ModelInstance::get_model_name() const {
  return model_name_;
}

const std::vector<unique_ptr<DrakeJoint>>& ModelInstance::get_joints() const {
  return joints_;
}

const std::vector<unique_ptr<RigidBody>>& ModelInstance::get_bodies() const {
  return bodies_;
}

const std::vector<unique_ptr<RigidBodyFrame>>& ModelInstance::get_frames() const {
  return frames_;
}

const std::vector<unique_ptr<RigidBodyLoop>>& ModelInstance::get_loops() const {
  return loops_;
}

const std::vector<unique_ptr<RigidBodyActuator>>& ModelInstance::get_actuators() const {
  return actuators_;
}

const std::vector<unique_ptr<RigidBodySensor>>& ModelInstance::get_sensors() const {
  return sensors_;
}

const std::vector<ModelInstance*>& ModelInstance::get_model_instances() const {
  return model_instances_;
}
}  // namespace plants
}  // namespace systems
}  // namespace drake
