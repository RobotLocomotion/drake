#pragma once

#include <limits>
#include <string>

#include "drake/common/drake_export.h"

class RigidBody;

/**
 * Defines a physical actuator (i.e., an electric motor and step-down
 * transmission) that operates on a joint. This class assumes the actuator
 * has a single DOF.
 */
class DRAKE_EXPORT RigidBodyActuator {
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
      const std::string& name, const RigidBody* body,
      double reduction = 1.0,
      double effort_limit_min = -std::numeric_limits<double>::infinity(),
      double effort_limit_max = std::numeric_limits<double>::infinity());

  const std::string name_;
  const RigidBody* const body_;
  const double reduction_;
  const double effort_limit_min_;
  const double effort_limit_max_;
};
