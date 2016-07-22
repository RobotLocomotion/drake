#pragma once

#include <limits>
#include <string>

#include "drake/drakeRBM_export.h"

class RigidBody;

class DRAKERBM_EXPORT RigidBodyActuator {
 public:
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
