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
class DRAKERBM_EXPORT RigidBodyElementId {
 public:
  /**
   * The constructor.
   *
   * @param[in] instance_name This must be unique to each call to the method
   * that adds a specification like a URDF or SDF to the `RigidBodySystem`.
   * Since multiple models may exist within a single specification, multiple
   * models may share the same instance name.
   *
   * @param[in] model_name The name of the model to which the rigid body element
   * belongs. Since a specification like URDF and SDF can be added multiple
   * times into the `RigidBodySystem`, there can be multiple models that share
   * the same model name.
   *
   * @param[in] element_name The name of the element within the model. These
   * could be, for example, the name of a joint, body, frame, loop joint,
   * actuator, sensor, etc.
   */
  RigidBodyElementId(
      const std::string& instance_name,
      const std::string& model_name,
      const std::string& element_name);

  /**
   * Returns a reference to the modeling element's instance name.
   */
  const std::string& get_instance_name() const;

  /**
   * Returns a reference to the modeling element's model name. This is the
   * name of the model to which the modeling element belongs.
   */
  const std::string& get_model_name() const;

  /**
   * Returns a reference to the modeling element's name.
   */
  const std::string& get_element_name() const;

 private:
  const std::string instance_name_;
  const std::string model_name_;
  const std::string element_name_;
};


bool operator==(const RigidBodyElementId& left_element,
    const RigidBodyElementId& right_element) {
    return
        left_element.get_instance_name() == right_element.get_instance_name() &&
        left_element.get_model_name() == right_element.get_model_name() &&
        left_element.get_element_name() == right_element.get_element_name();
}

bool operator!=(const RigidBodyElementId& left_element,
    const RigidBodyElementId& right_element) {
    return !(left_element == right_element);
}

}  // namespace plants
}  // namespace systems
}  // namespace drake
