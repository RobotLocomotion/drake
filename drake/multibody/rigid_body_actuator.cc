#include "drake/multibody/rigid_body_actuator.h"

RigidBodyActuator::RigidBodyActuator(const std::string& name,
                                     const RigidBody<double>* body,
                                     double reduction,
                                     double effort_limit_min,
                                     double effort_limit_max)
    : name_(name),
      body_(body),
      reduction_(reduction),
      effort_limit_min_(effort_limit_min),
      effort_limit_max_(effort_limit_max) {}
