#include "drake/automotive/gen/euler_floating_joint_state.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace automotive {

const int EulerFloatingJointStateIndices::kNumCoordinates;
const int EulerFloatingJointStateIndices::kX;
const int EulerFloatingJointStateIndices::kY;
const int EulerFloatingJointStateIndices::kZ;
const int EulerFloatingJointStateIndices::kRoll;
const int EulerFloatingJointStateIndices::kPitch;
const int EulerFloatingJointStateIndices::kYaw;

const std::vector<std::string>&
EulerFloatingJointStateIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "x", "y", "z", "roll", "pitch", "yaw",
      });
  return coordinates.access();
}

}  // namespace automotive
}  // namespace drake
