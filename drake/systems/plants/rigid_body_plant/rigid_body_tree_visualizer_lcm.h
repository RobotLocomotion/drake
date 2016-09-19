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
 * at startup, and run-time, which periodically runs for the duration of the
 * simulation.
 *
 * During initialization, this system block analyzes the RigidBodyTree and
 * tells Drake Visualizer what it will be visualizing. For example, these
 * include the number of rigid bodies and their dimensions. The LCM message used
 * during this phase is called `drake::lcmt_viewer_load_robot` and the channel
 * name is "DRAKE_VIEWER_LOAD_ROBOT".
 *
 * During run-time, this system block takes the current state of the model and
 * tells Drake Visualizer how to visualize the model. For example, this includes
 * the position and orientation of each rigid body mentioned during
 * initialization. The LCM messages used during this phase is
 * `drake::lcmt_viewer_draw` and the channel name is "DRAKE_VIEWER_DRAW".
 *
 * To support multiple simultaneous instances of this system, which may
 * occur when multiple unit tests are running in parallel, the channel names
 * can be modified by a postfix string, which is specified as an input parameter
 * to the constructor.
 *
 * @ingroup systems
 */
class DRAKE_RBP_EXPORT RigidBodyTreeVisualizerLcm : public LeafSystem<double> {
 public:
  /**
   * A constructor that initializes the input port from which the generalized
   * state is received, the translator that converts the generalized state of
   * the rigid body tree to LCM messages that are used by the Drake Visualizer
   * to visualize the tree, and the load messages that informs the Drake
   * Visualizer of what to visualize.
   *
   * @param[in] tree A reference to the rigid body tree that should be
   * visualized by Drake Visualizer. This reference must remain valid for the
   * duration of this object.
   *
   * @param[in] lcm A pointer to the object through which LCM messages can be
   * published. This pointer must remain valid for the duration of this object.
   */
  explicit RigidBodyTreeVisualizerLcm(const RigidBodyTree& tree,
      ::lcm::LCM* lcm);

  std::string get_name() const override;

  void EvalOutput(const systems::Context<double>& context,
                  systems::SystemOutput<double>* output) const override {}

  /**
   * Returns a reference to the drake::lcmt_viewer_load_robot message that was
   * transmitted by this system. This is intended to only be for unit testing
   * purposes.
   */
  const drake::lcmt_viewer_load_robot& get_load_message() const;

  /**
   * Returns a reference to the bytes of the most recently transmitted
   * drake::viewer_draw message. This is intended to only be for unit testing
   * purposes.
   */
  const std::vector<uint8_t>& get_draw_message_bytes() const;

 protected:
  void DoPublish(const systems::Context<double>& context) const override;

 private:
  // Initializes member variable load_message_.
  void InitializeLoadMessage(const RigidBodyTree& tree);

  // Sends a drake::lcmt_viewer_load_robot message containing a description of
  // what should be visualized. The message is intended to be received by the
  // Drake Visualizer. This is called once by the constructor.
  void PublishLoadRobot() const;

  // A pointer to the LCM subsystem. It is through this object that LCM messages
  // are published and message subscriptions are formed.
  ::lcm::LCM* const lcm_;

  // Using 'mutable' here is OK since it's only used for assertion checking.
  mutable bool sent_load_robot_{false};

  // The LCM load message to send to the Drake Visualizer.
  drake::lcmt_viewer_load_robot load_message_;

  // The translator that convert from the rigid body tree's generalized state
  // vector to an drake::lcmt_viewer_draw message.
  ViewerDrawTranslator draw_message_translator_;

  // Using 'mutable' here is OK since it's only used for unit testing checking.
  mutable std::vector<uint8_t> message_bytes_;
};

}  // namespace systems
}  // namespace drake
