#pragma once

#include "drake/lcm/drake_lcm_interface.h"
#include "drake/multibody/rigid_body_plant/viewer_draw_translator.h"

namespace drake {
namespace manipulation {

/**
 * A utility to render a `RigidBodyTree` in a specified configuration.
 */
class SimpleTreeVisualizer {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleTreeVisualizer)

  /**
   * Constructs the `SimpleTreeVisualizer` and publishes a
   * DRAKE_VIEWER_LOAD_ROBOT message. Note that the drake-visualizer must be
   * externally started before this constructor is called.
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

}  // namespace manipulation
}  // namespace drake
