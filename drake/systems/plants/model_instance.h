#pragma once

#include <string>
#include <vector>

#include "drake/drakeRBM_export.h"
// #include "drake/systems/plants/joints/DrakeJoint.h"
// #include "drake/systems/plants/rigid_body_loop.h"
// #include "drake/systems/plants/RigidBody.h"
// #include "drake/systems/plants/RigidBodyFrame.h"
// #include "drake/systems/plants/RigidBodySystem.h"

class DrakeJoint;
class RigidBody;
class RigidBodyActuator;
class RigidBodyFrame;
class RigidBodyLoop;
class RigidBodySensor;

namespace drake {
namespace systems {
namespace plants {


/**
 * An instance of a model that is being simulated. It owns pointers to all of
 * the modeling elements that constitute a particular instance of a model.
 * Modeling elements include:
 *
 *  - `DrakeJoint`
 *  - `RigidBody`
 *  - `RigidBodyFrame`
 *  - `RigidBodyLoop`
 *  - `RigidBodyActuator`
 *  - `RigidBodySensor`.
 *
 * In addition to the above, this class maintains a vector of pointers to other
 * `ModelInstance` objects to describe hierarchical models. For example, a
 * `ModelInstance` representing a hand may include pointers to five other
 * `ModelInstance` objects, each representing a different finger.
 */
class DRAKERBM_EXPORT ModelInstance {
 public:
  /**
   * A default constructor that uses a generic instance name.
   */
  ModelInstance();

  /**
   * A constructor.
   *
   * @param[in] model_instance_name The name of the model instance. This can be
   * any value and is provided purely for the user's benefit. It is recommended,
   * but not enforced, that all `ModelInstance` objects used by a particular
   * simulation have different instance names.
   */
  ModelInstance(const std::string& model_instance_name);

  /**
   * Sets the model name. Since multiple instance of a particular model may
   * exist within the same simulation, multiple `ModelInstance` objects may
   * share the same @p model_name.
   */
  void set_model_name(const std::string& model_name);

  /**
   * Adds a pointer to a `DrakeJoint` to this object. The pointer must
   * remain valid for the lifetime of this object.
   */
  void add_joint(const DrakeJoint* joint);

  /**
   * Adds a pointer to a `RigidBody` to this object. The pointer must
   * remain valid for the lifetime of this object.
   */
  void add_body(const RigidBody* body);

  /**
   * Adds a pointer to a `RigidBodyFrame` to this object. The pointer must
   * remain valid for the lifetime of this object.
   */
  void add_frame(const RigidBodyFrame* frame);

  /**
   * Adds a pointer to a `RigidBodyLoop` to this object. The pointer must
   * remain valid for the lifetime of this object.
   */
  void add_loop(const RigidBodyLoop* loop);

  /**
   * Adds a pointer to a `RigidBodyActuator` to this object. The pointer
   * must remain valid for the lifetime of this object.
   */
  void add_actuator(const RigidBodyActuator* actuator);

  /**
   * Adds a pointer to a `RigidBodySensor` to this object. The pointer must
   * remain valid for the lifetime of this object.
   */
  void add_sensor(const RigidBodySensor* sensor);

  /**
   * Adds a pointer to a `ModelInstance` to this object. This pointer
   * must remain valid throughout this object's lifetime.
   */
  void add_model_instance(ModelInstance* model_instance);

  /**
   * Returns the model instance name.
   */
  const std::string& get_model_intance_name() const;

  /**
   * Returns the model name.
   */
  const std::string& get_model_name() const;

  /**
   * Returns the `DrakeJoint` objects that are part of this model instance.
   */
  const std::vector<const DrakeJoint*>& get_joints() const;

  /**
   * Returns the `RigidBody` objects that are part of this model instance.
   */
  const std::vector<const RigidBody*>& get_bodies() const;

  /**
   * Returns the `RigidBodyFrame` objects that are part of this model instance.
   */
  const std::vector<const RigidBodyFrame*>& get_frames() const;

  /**
   * Returns the `RigidBodyLoop` objects that are part of this model instance.
   */
  const std::vector<const RigidBodyLoop*>& get_loops() const;

  /**
   * Returns the `RigidBodyActuator` objects that are part of this model
   * instance.
   */
  const std::vector<const RigidBodyActuator*>& get_actuators() const;

  /**
   * Returns the `RigidBodySensor` objects that are part of this model instance.
   */
  const std::vector<const RigidBodySensor*>& get_sensors() const;

  /**
   * Returns the `ModelInstance` objects that are part of this model instance.
   */
  const std::vector<ModelInstance*>& get_model_instances() const;

 private:
  const std::string model_instance_name_;

  std::string model_name_;

  std::vector<const DrakeJoint*> joints_;

  std::vector<const RigidBody*> bodies_;

  std::vector<const RigidBodyFrame*> frames_;

  std::vector<const RigidBodyLoop*> loops_;

  std::vector<const RigidBodyActuator*> actuators_;

  std::vector<const RigidBodySensor*> sensors_;

  std::vector<ModelInstance*> model_instances_;
};

}  // namespace plants
}  // namespace systems
}  // namespace drake
