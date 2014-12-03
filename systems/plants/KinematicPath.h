#ifndef KINEMATICPATH_H_
#define KINEMATICPATH_H_

#include <vector>
#include <memory>

struct KinematicPath
{
  std::vector<int> joint_path;
  std::vector<int> joint_direction_signs;
  std::vector<int> body_path;
};


#endif /* KINEMATICPATH_H_ */
