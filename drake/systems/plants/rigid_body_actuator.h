#pragma once

#include <limits>
#include <string>

#include "drake/drakeRBM_export.h"

class RigidBody;

class DRAKERBM_EXPORT RigidBodyActuator {
 public:
  RigidBodyActuator(
      const std::string& name, const RigidBody* body,
      double reduction = 1.0,
      double effort_limit_min_in = -std::numeric_limits<double>::infinity(),
      double effort_limit_max_in = std::numeric_limits<double>::infinity());

  const std::string name_;
  const RigidBody* const body_;
  const double reduction_;
  const double effort_limit_min_;
  const double effort_limit_max_;
};
