#pragma once

#include <vector>

#include "drake/common/drake_export.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/rigid_body_plant/viewer_draw_translator.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/lcmt_viewer_draw.hpp"

namespace drake {
namespace systems {

// TODO(liang.fok) Update the class's description once System 2.0 allows systems
// to declare that they need a certain action to be performed at simulation time
// t_0.
/**
 * This is a Drake System 2.0 block that takes a RigidBodyTree and publishes LCM
 * messages that are intended for the Drake Visualizer. It does this in two
 * phases: initialization, which runs when DoPublish() is called with
 * Context::get_time() equal to zero, and run-time, which runs every time
 * DoPublish() is called.
 *
 * During initialization, this system block analyzes the RigidBodyTree and tells
 * Drake Visualizer what it will be visualizing. For example, these include the
 * number of rigid bodies, their dimensions, and colors. The LCM message used
 * during this phase is called `lcmt_viewer_load_robot` and the channel name is
 * "DRAKE_VIEWER_LOAD_ROBOT".
 *
 * During run-time, this system block takes the current state of the model and
 * tells Drake Visualizer how to visualize the model. For example, this includes
 * the position and orientation of each rigid body. The LCM messages used during
 * this phase is `lcmt_viewer_draw` and the channel name is
 * "DRAKE_VIEWER_DRAW".
 *
 * @ingroup rigid_body_systems
 */
class DRAKE_EXPORT DrakeVisualizer
    : public LeafSystem<double> {
 public:
  /**
   * A constructor that prepares for the transmission of `lcmt_viewer_draw` and
   * `lcmt_viewer_load_robot` messages, but does not actually publish anything.
   * LCM message publications occur each time DrakeVisualizer::Publish() is
   * called.
   *
   * @param[in] tree A reference to the rigid body tree that should be
   * visualized by Drake Visualizer. This reference must remain valid for the
   * duration of this object.
   *
   * @param[in] lcm A pointer to the object through which LCM messages can be
   * published. This pointer must remain valid for the duration of this object.
   */
  DrakeVisualizer(const RigidBodyTree& tree,
      drake::lcm::DrakeLcmInterface* lcm);

  void EvalOutput(const systems::Context<double>& context,
                  systems::SystemOutput<double>* output) const override {}

  /**
   * Returns a reference to the lcmt_viewer_load_robot message that was
   * transmitted by this system. This is intended to be for unit testing
   * purposes only.
   */
  const lcmt_viewer_load_robot& get_load_message() const;

  /**
   * Returns a reference to the bytes of the most recently transmitted
   * lcmt_viewer_draw message. This is intended to be for unit testing purposes
   * only.
   */
  const std::vector<uint8_t>& get_draw_message_bytes() const;

 protected:
  void DoPublish(const systems::Context<double>& context) const override;

 private:
  // Returns a partially-initialized lcmt_viewer_load_robot message. After this
  // method is called, all dynamically-sized member variables are correctly
  // sized, and the names and model instance IDs of the rigid bodies are set
  // within load_message_.
  static lcmt_viewer_load_robot CreateLoadMessage(
      const RigidBodyTree& tree);

  // Publishes a lcmt_viewer_load_robot message containing a description
  // of what should be visualized. The message is intended to be received by the
  // Drake Visualizer.
  void PublishLoadRobot() const;

  // A pointer to the LCM subsystem. It is through this object that LCM messages
  // are published.
  drake::lcm::DrakeLcmInterface* const lcm_;

  // Using 'mutable' here is OK since it's only used for assertion checking.
  mutable bool sent_load_robot_{false};

  // The LCM load message to send to the Drake Visualizer.
  const lcmt_viewer_load_robot load_message_;

  // The translator that converts from the RigidBodyTree's generalized state
  // vector to a lcmt_viewer_draw message.
  const ViewerDrawTranslator draw_message_translator_;

  // TODO(liang.fok) Remove this once this class is updated to support LCM mock
  // interfaces and dependency injection. See #3546.
  //
  // Using 'mutable' here is OK since it's only used for unit test checking.
  mutable std::vector<uint8_t> draw_message_bytes_;
};

}  // namespace systems
}  // namespace drake
