#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {

/**
 * This system takes in a Context<double> for the corresponding MultibodyPlant,
 * and outputs a drake::lcmt_viewer_draw message for visualizing the requested
 * frames.
 *
 * @system{KinematicsCaculator,
 *   @input_port(kinematics)
 *   @output_port(visualization_message)
 * }
 */
class FrameVisualizer : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FrameVisualizer)

  /**
   * Constructor.
   * @param plant A const pointer to the MultibodyPlant. It is aliased
   * internally, so its life span must be longer than this.
   * @param frames A vector of frames in @p plant for visualization.
   */
  FrameVisualizer(const MultibodyPlant<double>* plant,
                  const std::vector<const Frame<double>*>& frames);

 private:
  void CalcOutputMsg(const systems::Context<double>& context,
                     lcmt_viewer_draw* msg) const;

  const MultibodyPlant<double>& plant_;
  const std::vector<const Frame<double>*>& frames_;
};

}  // namespace multibody
}  // namespace drake
