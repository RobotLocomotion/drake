#include "drake/systems/plants/rigid_body_actuator.h"

RigidBodyActuator::RigidBodyActuator(
      const std::string& name_in, const RigidBody* body_in, double reduction_in,
      double effort_limit_min_in, double effort_limit_max_in)
      : name(name_in),
        body(body_in),
        reduction(reduction_in),
        effort_limit_min(effort_limit_min_in),
        effort_limit_max(effort_limit_max_in) {
}
