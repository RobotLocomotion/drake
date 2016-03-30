#ifndef DRAKE_SYSTEMS_PLANTS_KINEMATICPATH_H_
#define DRAKE_SYSTEMS_PLANTS_KINEMATICPATH_H_

#include <vector>
#include <memory>

struct KinematicPath {
  std::vector<int> joint_path;
  std::vector<int> joint_direction_signs;
  std::vector<int> body_path;
};

#endif  // DRAKE_SYSTEMS_PLANTS_KINEMATICPATH_H_
