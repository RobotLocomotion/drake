#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/**
 * This is a Drake System block that takes in a state vector, and outputs a
 * drake::lcmt_viewer_draw message that contains information about the frames
 * for visualization.
 *
 * @ingroup rigid_body_systems
 */
class FrameVisualizer : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FrameVisualizer)

  /**
   * Constructor.
   * @param tree A const pointer to the RigidBodyTree used for doing kinematics.
   *        Its life span must be longer than this.
   * @param local_transforms A vector of frames to be visualized.
   * @param lcm A pointer to the object through which LCM messages can be
   *        published. This pointer must remain valid for the duration of this
   *        object.
   */
  FrameVisualizer(const RigidBodyTree<double>* tree,
                  const std::vector<RigidBodyFrame<double>>& local_transforms,
                  drake::lcm::DrakeLcmInterface* lcm);

  /**
   * Sets the publishing period of this system. See
   * LeafSystem::DeclarePublishPeriodSec() for details about the semantics of
   * parameter `period`.
   */
  void set_publish_period(double period) {
    this->DeclarePeriodicPublishEvent(period, 0.,
        &FrameVisualizer::PublishFramePose);
    publish_per_step_ = false;  // Disable default per-step publish.
  }

  /**
   * Sets the LCM channel to which frames are published.
   */
  void set_lcm_channel(const std::string& lcm_channel) {
    lcm_channel_ = lcm_channel;
  }

 private:
  EventStatus PerStepPublishFramePose(
      const systems::Context<double>& context) const {
    if (!publish_per_step_) return EventStatus::DidNothing();
    return PublishFramePose(context);
  }

  EventStatus PublishFramePose(
      const systems::Context<double>& context) const;

  const RigidBodyTree<double>& tree_;
  drake::lcm::DrakeLcmInterface* const lcm_;
  const std::vector<RigidBodyFrame<double>> local_transforms_;

  drake::lcmt_viewer_draw default_msg_{};
  std::string lcm_channel_{"DRAKE_DRAW_FRAMES"};

  // Visualize every step by default; disabled if a period is specified.
  bool publish_per_step_{true};
};

}  // namespace systems
}  // namespace drake
