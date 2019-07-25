#include <fstream>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "robotlocomotion/robot_plan_t.hpp"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace planar_gripper {

using drake::multibody::MultibodyPlant;
using drake::math::RigidTransform;
using drake::math::RollPitchYaw;

template<typename T>
void WeldGripperFrames(multibody::MultibodyPlant<T> *plant) {
  // This function is copied and adapted from planar_gripper_simulation.py
  const double outer_radius = 0.19;
  const double f1_angle = M_PI / 3;
  const math::RigidTransformd XT(math::RollPitchYaw<double>(0, 0, 0),
                                 Eigen::Vector3d(0, 0, outer_radius));

  // Weld the first finger.
  math::RigidTransformd X_PC1(math::RollPitchYaw<double>(f1_angle, 0, 0),
                              Eigen::Vector3d::Zero());
  X_PC1 = X_PC1 * XT;
  const multibody::Frame<T> &finger1_base_frame =
      plant->GetFrameByName("finger1_base");
  plant->WeldFrames(plant->world_frame(), finger1_base_frame, X_PC1);

  // Weld the second finger.
  const math::RigidTransformd X_PC2 =
      math::RigidTransformd(math::RollPitchYawd(M_PI / 3 * 2, 0, 0),
                            Eigen::Vector3d::Zero()) *
          X_PC1;
  const multibody::Frame<T> &finger2_base_frame =
      plant->GetFrameByName("finger2_base");
  plant->WeldFrames(plant->world_frame(), finger2_base_frame, X_PC2);

  // Weld the 3rd finger.
  const math::RigidTransformd X_PC3 =
      math::RigidTransformd(math::RollPitchYawd(M_PI / 3 * 2, 0, 0),
                            Eigen::Vector3d::Zero()) *
          X_PC2;
  const multibody::Frame<T> &finger3_base_frame =
      plant->GetFrameByName("finger3_base");
  plant->WeldFrames(plant->world_frame(), finger3_base_frame, X_PC3);
}
}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
