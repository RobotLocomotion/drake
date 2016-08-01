#pragma once

#include <string>

#include "drake/drakeRBM_export.h"

namespace drake {
namespace systems {
namespace plants {

/**
 * Identifies a unique modeling element within a `RigidBodySystem`. Modeling
 * elements include the name of a `DrakeJoint`, `RigidBody`, `RigidBodyFrame`,
 * `RigidBodyLoop`, `RigidBodyActuator`, and `RigidBodySensor`.
 *
 * Instances of this class are used to access a unique modeling element within
 * the RigidBodySystem. Note that it includes multiple member variables. None of
 * the individual member variables guarantee a unique match to a single rigid
 * body element within a `RigidBodySystem`. A unique match is only guaranteed
 * using the union of all member variables in this class.
 */
class DRAKERBM_EXPORT ModelElementId {
 public:
  // TODO(liang.fok) Remove this default constructor once the setters are
  // removed. See: https://github.com/RobotLocomotion/drake/issues/2990
  ModelElementId() {}

  // TODO(liang.fok) Remove model_instance_id. See:
  // https://github.com/RobotLocomotion/drake/issues/2973
  /**
   * The constructor.
   *
   * @param[in] model_instance_name This must be unique to each call to the
   * method that adds a specification like a URDF or SDF to the
   * `RigidBodySystem`. Each model must have a unique instance name even if
   * multiple models exist within a single specification (as is the case for
   * SDF). Multiple instances of the same model may *not* share the same
   * instance name.
   *
   * @param[in] model_name The name of the model to which the rigid body element
   * belongs. Since a specification like URDF and SDF can be added multiple
   * times into the `RigidBodySystem`, there can be multiple models that share
   * the same model name.
   *
   * @param[in] element_name The name of the element within a model.
   * These could be, for example, the name of a joint, body, frame, loop joint,
   * actuator, sensor, etc.
   *
   * @param[in] model_instance_id A unique number that identifies the model
   * instance. This is included for legacy support and will be removed in the
   * near future.
   */
  ModelElementId(const std::string& model_instance_name,
                 const std::string& model_name,
                 const std::string& element_name,
                 int model_instance_id);

  /**
   * Returns a reference to the modeling element's instance name.
   */
  const std::string& get_model_instance_name() const;

  /**
   * Returns a reference to the modeling element's model name. This is the
   * name of the model to which the modeling element belongs.
   */
  const std::string& get_model_name() const;

  /**
   * Returns a reference to the modeling element's name.
   */
  const std::string& get_element_name() const;

  // TODO(liang.fok) Remove this method. See:
  // https://github.com/RobotLocomotion/drake/issues/2973
  /**
   * Returns the model instance ID. This is a unique number for each model
   * instance.
   *
   * Note that this is included for legacy support and will be removed in the
   * near future.
   */
  int get_model_instance_id() const;

  // TODO(liang.fok) Remove this method. See:
  // https://github.com/RobotLocomotion/drake/issues/2990
  void set_model_instance_name(const std::string& model_name);

  // TODO(liang.fok) Remove this method. See:
  // https://github.com/RobotLocomotion/drake/issues/2990
  void set_model_name(const std::string& model_name);

  // TODO(liang.fok) Remove this method. See:
  // https://github.com/RobotLocomotion/drake/issues/2990
  void set_element_name(const std::string& model_name);

  // TODO(liang.fok) Remove this method. See:
  // https://github.com/RobotLocomotion/drake/issues/2990
  void set_model_instance_id(int model_instance_id);

 private:
  // TODO(liang.fok) Make these variables const once the setters are removed.
  // See: https://github.com/RobotLocomotion/drake/issues/2990
  std::string model_instance_name_;
  std::string model_name_;
  std::string element_name_;

  // TODO(liang.fok) Remove model_instance_id_. See:
  // https://github.com/RobotLocomotion/drake/issues/2973
  int model_instance_id_{0};
};

DRAKERBM_EXPORT
bool operator==(const ModelElementId& left_element,
                const ModelElementId& right_element);

DRAKERBM_EXPORT
bool operator!=(const ModelElementId& left_element,
                const ModelElementId& right_element);

}  // namespace plants
}  // namespace systems
}  // namespace drake
