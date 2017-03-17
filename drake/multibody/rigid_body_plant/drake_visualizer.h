#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/multibody/rigid_body_plant/viewer_draw_translator.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/output_port_value.h"
#include "drake/systems/primitives/signal_log.h"

namespace drake {
namespace systems {

// TODO(liang.fok) Update the class's description once System 2.0 allows systems
// to declare that they need a certain action to be performed at simulation time
// t_0.
/**
 * This is a Drake System 2.0 block that takes a RigidBodyTree and publishes LCM
 * messages that are intended for the DrakeVisualizer. It does this in two
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
 * The visualizer has an option that causes it to save the state it dispatches
 * for drawing and allows replay of that cached data at wall clock time --
 * i.e., one second of simulation is played back for every second in the real
 * world.  The playback *rate* is currently capped at 60 Hz.  This is useful
 * for immediate review of simulations which evaluate at time rates radically
 * out of scale with wall clock time, enabling intuitive understanding of the
 * simulation results.  See ReplayCachedSimulation().
 *
 * @ingroup rigid_body_systems
 */
class DrakeVisualizer : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrakeVisualizer)

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
   *
   * @param[in] enable_playback  If true, the visualizer will cache the
   * input data for playback and ReplayCachedSimulation() will replay that
   * cache data.
   */
  DrakeVisualizer(const RigidBodyTree<double>& tree,
                  drake::lcm::DrakeLcmInterface* lcm,
                  bool enable_playback = false);

  /**
   * Sets the publishing period of this system. See
   * LeafSystem::DeclarePublishPeriodSec() for details about the semantics of
   * parameter `period`.
   */
  void set_publish_period(double period);

  // TODO(SeanCurtis-TRI): Optional features:
  //    1. Specify number of loops (<= 0 --> infinite looping)
  //    2. Specify range of playback [start, end] for cached data from times
  //       in the range [0, T], such that start < end, start >= 0 and
  //       end <= T.  (Although, putting end > T *is* valid, it would
  //       manifest as a *pause* at the end of the playback before finishing.
  //    3. Optionally force the replay to emit the messages to load the
  //       geometry again.
  //    4. Specify playback rate.
  //    5. Add a wall-clock scale factor; e.g., play faster than real time,
  //       slower than real time, etc.
  /**
   * Causes the visualizer to playback its cached data at real time.  If it has
   * not been configured to record/playback, a warning message will be written
   * to the log, but otherwise, no work will be done.
   */
  void ReplayCachedSimulation() const;

  /**
   * Plays back (at real time) a trajectory representing the input signal.
   */
  void PlaybackTrajectory(
      const PiecewisePolynomial<double>& input_trajectory) const;

 private:
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override {}

  void DoPublish(const systems::Context<double>& context) const override;

  // Returns a partially-initialized lcmt_viewer_load_robot message. After this
  // method is called, all dynamically-sized member variables are correctly
  // sized, and the names and model instance IDs of the rigid bodies are set
  // within load_message_.
  static lcmt_viewer_load_robot CreateLoadMessage(
      const RigidBodyTree<double>& tree);

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

  // The (optional) log used for recording and playback.
  std::unique_ptr<SignalLog<double>> log_{nullptr};
};

}  // namespace systems
}  // namespace drake
