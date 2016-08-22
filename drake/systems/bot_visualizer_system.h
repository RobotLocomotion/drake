#pragma once

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"
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
 */
class DRAKELCMSYSTEM2_EXPORT BotVisualizerSystem : public System<double> {
 public:
  /**
   * A constructor that initializes the Drake visualizer by informing it of all
   * the bodies within @p tree that need to be visualized.
   *
   * @param[in] tree The rigid body tree that's should be visualized. This
   * reference must remain valid for the lifetime of this object.
   *
   * @param[in] lcm A pointer to the LCM subsystem.
   */
  BotVisualizerSystem(const RigidBodyTree& tree, ::lcm::LCM* lcm);


  ~BotVisualizerSystem() override;

  // Disable copy and assign.
  BotVisualizerSystem(const BotVisualizerSystem&) = delete;
  BotVisualizerSystem& operator=(const BotVisualizerSystem&) = delete;

  std::string get_name() const override;

  /**
   * The default context for this system is one that has one input port and
   * no state.
   */
  std::unique_ptr<ContextBase<double>> CreateDefaultContext() const override;

  /**
   * The output contains zero ports.
   */
  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const ContextBase<double>& context) const override;

  /**
   * Takes the VectorInterface from the input port of the context, computes the
   * pose of each rigid body in the RigidBodyTree that was provided as an input
   * argument to the constructor, and publishes draw messages for visualizing
   * the current state of the model in Drake Visualizer.
   */
  void EvalOutput(const ContextBase<double>& context,
                  SystemOutput<double>* output) const override;

 private:

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
  mutable drake::lcmt_viewer_draw draw_msg_;
};

}  // namespace systems
}  // namespace drake
