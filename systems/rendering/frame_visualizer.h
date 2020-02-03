#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace rendering {

/**
 * This system takes in a Context<T> for the corresponding MultibodyPlant,
 * and outputs a drake::lcmt_viewer_draw message for visualizing the requested
 * frames with drake_visualizer.
 *
 * @system{FrameVisualizer,
 *   @input_port(kinematics)
 *   @output_port(visualization_message)
 * }
 */
template <typename T>
class FrameVisualizer final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FrameVisualizer)

  /**
   * Constructor.
   * @param plant A const pointer to the MultibodyPlant. It is aliased
   * internally, so its life span must be longer than this.
   * @param frames A vector of frames in @p plant for visualization.
   */
  FrameVisualizer(const multibody::MultibodyPlant<T>* plant,
                  const std::vector<const multibody::Frame<T>*>& frames);

 private:
  void CalcOutputMsg(const Context<T>& context, lcmt_viewer_draw* msg) const;

  const multibody::MultibodyPlant<T>& plant_;
  const std::vector<const multibody::Frame<T>*>& frames_;
};

}  // namespace rendering
}  // namespace systems
}  // namespace drake
