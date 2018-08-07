#include "drake/examples/allegro_hand/allegro_common.h"

#include "drake/multibody/parsers/urdf_parser.h"

using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace allegro_hand {

void CreateTreeFromFixedModelAtPose(const std::string& model_file_name,
                                     RigidBodyTreed* tree,
                                     const Vector3d& position /* on base*/,
                                     const Vector3d& orientation) {
  auto weld_to_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world",
      nullptr, position, orientation);  

  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      model_file_name, drake::multibody::joints::kFixed,
      weld_to_frame, tree);
}

}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake
