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

void ModelInstance::add_joint(const DrakeJoint* joint) {
  joints_.push_back(joint);
}

void ModelInstance::add_body(const RigidBody* body) {
  bodies_.push_back(body);
}

void ModelInstance::add_frame(const RigidBodyFrame* frame) {
  frames_.push_back(frame);
}

void ModelInstance::add_loop(const RigidBodyLoop* loop) {
  loops_.push_back(loop);
}

void ModelInstance::add_actuator(const RigidBodyActuator* actuator) {
  actuators_.push_back(actuator);
}

void ModelInstance::add_sensor(const RigidBodySensor* sensor) {
  sensors_.push_back(sensor);
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

const std::vector<const DrakeJoint*>& ModelInstance::get_joints() const {
  return joints_;
}

const std::vector<const RigidBody*>& ModelInstance::get_bodies() const {
  return bodies_;
}

const std::vector<const RigidBodyFrame*>& ModelInstance::get_frames() const {
  return frames_;
}

const std::vector<const RigidBodyLoop*>& ModelInstance::get_loops() const {
  return loops_;
}

const std::vector<const RigidBodyActuator*>& ModelInstance::get_actuators() const {
  return actuators_;
}

const std::vector<const RigidBodySensor*>& ModelInstance::get_sensors() const {
  return sensors_;
}

const std::vector<ModelInstance*>& ModelInstance::get_model_instances() const {
  return model_instances_;
}
}  // namespace plants
}  // namespace systems
}  // namespace drake
