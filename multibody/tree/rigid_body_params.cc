#include "drake/multibody/tree/rigid_body_params.h"

namespace drake {
namespace multibody {
namespace internal {

const int RigidBodyParamsIndex::kNumCoordinates;
// The index of each individual coordinate.
const int RigidBodyParamsIndex::k_mass;
const int RigidBodyParamsIndex::k_com_x;
const int RigidBodyParamsIndex::k_com_y;
const int RigidBodyParamsIndex::k_com_z;
const int RigidBodyParamsIndex::k_ixx;
const int RigidBodyParamsIndex::k_iyy;
const int RigidBodyParamsIndex::k_izz;
const int RigidBodyParamsIndex::k_ixy;
const int RigidBodyParamsIndex::k_ixz;
const int RigidBodyParamsIndex::k_iyz;

const std::vector<std::string>& RigidBodyParamsIndex::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "mass",
          "center_of_mass_x",
          "center_of_mass_y",
          "center_of_mass_z",
          "moment_of_inertia_Ixx",
          "moment_of_inertia_Iyy",
          "moment_of_inertia_Izz",
          "product_of_inertia_Ixy",
          "product_of_inertia_Ixz",
          "product_of_inertia_Iyz",
      });
  return coordinates.access();
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
