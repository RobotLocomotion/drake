#pragma once

#include "drake/lcm/drake_lcm_interface.h"
#include "drake/multibody/rigid_body_plant/viewer_draw_translator.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace tools {
// TODO(naveenoid) : Consider a more common location for this tool since its
// not Kuka IIWA specific.
/**
 * A utility to render a `RigidBodyTree` in a specified configuration in the
 * `drake-visualizer`.
 */
class SimpleTreeVisualizer {
 public:
  /**
   * Constructs the `SimpleTreeVisualizer`
   * @param tree constant reference to the `RigidBodyTree` that is to
   * be visualized. This reference must remain valid for the life time of this
   * class.
   * @param lcm pointer to the `lcm::DrakeLcm` interface.
   */
  SimpleTreeVisualizer(const RigidBodyTreed& tree, lcm::DrakeLcmInterface* lcm);

  /**
   * Visualizes a given state position configuration.
   * @param position_vector : A `VectorX` with the positions that are
   * to be visualized. The dimension of the position_vector must match the
   * number of positions in the `RigidBodyTree` with which this class was
   * constructed.
   */
  void visualize(const VectorX<double>& position_vector);

 private:
  const RigidBodyTreed& tree_;
  const int state_dimension_{0};
  const systems::ViewerDrawTranslator draw_message_translator_;
  lcm::DrakeLcmInterface* lcm_{nullptr};
};

}  // namespace tools
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
