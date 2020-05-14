#pragma once

#include <memory>
#include <vector>

#include "drake/attic_warning.h"

struct KinematicPath {
  std::vector<int> joint_path;
  std::vector<int> joint_direction_signs;
  std::vector<int> body_path;
};
