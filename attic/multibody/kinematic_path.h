#pragma once

#include <memory>
#include <vector>

struct KinematicPath {
  std::vector<int> joint_path;
  std::vector<int> joint_direction_signs;
  std::vector<int> body_path;
};
