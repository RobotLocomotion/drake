#pragma once

#include <limits>
#include <string>

#include "drake/drakeRBM_export.h"

class RigidBody;

/**
 * Defines certain properties of an actuator that operates on a joint.
 *
 * For more information, see:
 *  - http://wiki.ros.org/urdf/XML/Transmission#A.3Ctransmission.3E_Elements
 *  - http://sdformat.org/spec?ver=1.6&elem=joint
 */
class DRAKERBM_EXPORT RigidBodyActuator {
 public:
  /**
   * The constructor.
   *
   * @param[in] name_in The name of the actuator.
   *
   * @param[in] body_in A pointer to the rigid body whose joint's actuator is
   * being described by this class.
   *
   * @param[in] reduction_in The gear reduction ratio of the actuator.
   *
   * @param[in] effort_limit_min_in The actuator's minimum effort limit. This
   * assumes the actuator is effort (i.e., force/torque) controlled.
   *
   * @param[in] effort_limit_max_in The actuator's maximum effort limit. This
   * assumes the actuator is effort (i.e., force/torque) controlled.
   */
  RigidBodyActuator(
      const std::string& name_in, const RigidBody* body_in,
      double reduction_in = 1.0,
      double effort_limit_min_in = -std::numeric_limits<double>::infinity(),
      double effort_limit_max_in = std::numeric_limits<double>::infinity());

  const std::string name;
  const RigidBody* const body;
  const double reduction;
  const double effort_limit_min;
  const double effort_limit_max;
};
