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
    this->DeclarePeriodicPublish(period);
  }

 private:
  void DoPublish(
      const systems::Context<double>& context,
      const std::vector<const PublishEvent<double>*>&) const override;

  const RigidBodyTree<double>& tree_;
  drake::lcm::DrakeLcmInterface* const lcm_;
  const std::vector<RigidBodyFrame<double>> local_transforms_;

  drake::lcmt_viewer_draw default_msg_{};
};

}  // namespace systems
}  // namespace drake
