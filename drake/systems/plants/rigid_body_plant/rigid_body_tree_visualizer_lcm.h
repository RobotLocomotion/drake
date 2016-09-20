#pragma once

#include <lcm/lcm-cpp.hpp>

#include "drake/drake_rbp_export.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/rigid_body_plant/viewer_draw_translator.h"
#include "lcmtypes/drake/lcmt_viewer_load_robot.hpp"
#include "lcmtypes/drake/lcmt_viewer_draw.hpp"

namespace drake {
namespace systems {

/**
 * This is a Drake System 2.0 block that communicates with the Drake Visualizer
 * via LCM messages. It does this in two phases: initialization, which runs once
 * at startup, and run-time, which runs every simulation cycle for the duration
 * of the simulation.
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
 * @ingroup systems
 */
class DRAKE_RBP_EXPORT RigidBodyTreeVisualizerLcm : public LeafSystem<double> {
 public:
  /**
   * A constructor that prepares for the tranmission of `lcmt_viewer_load_robot`
   * and `lcmt_viewer_draw` messages, but does not actually publish anything.
   * LCM message publications occur each time
   * RigidBodyTreeVisualizerLcm::Publish() is called.
   *
   * @param[in] tree A reference to the rigid body tree that should be
   * visualized by Drake Visualizer. This reference must remain valid for the
   * duration of this object.
   *
   * @param[in] lcm A pointer to the object through which LCM messages can be
   * published. This pointer must remain valid for the duration of this object.
   */
  RigidBodyTreeVisualizerLcm(const RigidBodyTree& tree, ::lcm::LCM* lcm);

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
  // Returns a particially-initialized lcmt_viewer_load_robot message.
  static lcmt_viewer_load_robot InitializeLoadMessage(
      const RigidBodyTree& tree);

  // Publishes a lcmt_viewer_load_robot message containing a description
  // of what should be visualized. The message is intended to be received by the
  // Drake Visualizer.
  void PublishLoadRobot() const;

  // A pointer to the LCM subsystem. It is through this object that LCM messages
  // are published.
  ::lcm::LCM* const lcm_;

  // Using 'mutable' here is OK since it's only used for assertion checking.
  mutable bool sent_load_robot_{false};

  // The LCM load message to send to the Drake Visualizer.
  lcmt_viewer_load_robot load_message_;

  // The translator that converts from the RigidBodyTree's generalized state
  // vector to a lcmt_viewer_draw message.
  const ViewerDrawTranslator draw_message_translator_;

  // Using 'mutable' here is OK since it's only used for unit test checking.
  mutable std::vector<uint8_t> draw_message_bytes_;
};

}  // namespace systems
}  // namespace drake
