#pragma once

#include <limits>
#include <string>


template <typename T>
class RigidBody;

/**
 * Defines a physical actuator (i.e., an electric motor and step-down
 * transmission) that operates on a joint. This class assumes the actuator
 * has a single DOF.
 */
class RigidBodyActuator {
 public:
  /**
   * The constructor.
   *
   * @param[in] name The name of the actuator.
   *
   * @param[in] body A pointer to the rigid body whose joint's actuator is
   * being described by this class.
   *
   * @param[in] reduction The gear reduction ratio of the actuator.
   *
   * @param[in] effort_limit_min The actuator's minimum effort limit. This
   * has units of Nm for revolute joints and N for prismatic joints.
   *
   * @param[in] effort_limit_max The actuator's maximum effort limit. This
   * has units of Nm for revolute joints and N for prismatic joints.
   */
  RigidBodyActuator(
      const std::string& name, const RigidBody<double>* body,
      double reduction = 1.0,
      double effort_limit_min = -std::numeric_limits<double>::infinity(),
      double effort_limit_max = std::numeric_limits<double>::infinity());

  /**
   * Compares this %RigidBodyActuator with a clone. Since this method is
   * intended to compare a clone, an *exact* match is performed. This method
   * will only return `true` if the provided `other` %RigidBodyActuator is
   * exactly the same as this %RigidBodyActuator.
   */
  bool CompareToClone(const RigidBodyActuator& other) const;

  const std::string name_;
  const RigidBody<double>* const body_;
  const double reduction_;
  const double effort_limit_min_;
  const double effort_limit_max_;
};
