#pragma once

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeBotVisualizerSystem_export.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/plants/RigidBodyTree.h"

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
 */
class DRAKEBOTVISUALIZERSYSTEM_EXPORT BotVisualizerSystem :
    public LeafSystem<double> {
 public:
  /**
   * A constructor that initializes the Drake visualizer by informing it of all
   * the bodies within @p tree that need to be visualized.
   *
   * @param[in] tree The rigid body tree that's should be visualized. This
   * reference must remain valid for the lifetime of this object.
   *
   * @param[in] lcm A pointer to the LCM subsystem.
   *
   * @param[in] channel_postfix A postfix that is appended to the end of the LCM
   * channel names used by this system.
   */
  BotVisualizerSystem(const RigidBodyTree& tree, ::lcm::LCM* lcm,
    std::string channel_postfix = "");


  ~BotVisualizerSystem() override;

  // Disable copy and assign.
  BotVisualizerSystem(const BotVisualizerSystem&) = delete;
  BotVisualizerSystem& operator=(const BotVisualizerSystem&) = delete;

  std::string get_name() const override;

  /**
   * This System has no output ports so EvalOutput() does nothing.
   */
  void EvalOutput(const ContextBase<double>& context,
                  SystemOutput<double>* output) const override {}

 private:
  /**
   * Takes the VectorInterface from the input port of the context, computes the
   * pose of each rigid body in the RigidBodyTree that was provided as an input
   * argument to the constructor, and publishes draw messages for visualizing
   * the current state of the model in Drake Visualizer.
   */
  void DoPublish(const ContextBase<double>& context) const override;

  // Sends the drake::lcmt_viewer_load_robot messages to the Drake Visualizer
  // informing it of what needs to be visualized. This is called once by the
  // constructor.
  void initialize_drake_visualizer();

  // Initializes member variable draw_msg_.
  void initialize_draw_message();

  // A const reference to the RigidBodyTree whose rigid bodies are to be
  // visualized.
  const RigidBodyTree& tree_;

  // A pointer to the LCM subsystem.
  ::lcm::LCM* lcm_;

  // The LCM draw message to send to the Drake Visualizer. This member variable
  // is declared mutable so it can be modified by EvalOutput().
  drake::lcmt_viewer_draw draw_msg_;

  // The postfix of the channel names.
  std::string channel_postfix_;
};

}  // namespace systems
}  // namespace drake
