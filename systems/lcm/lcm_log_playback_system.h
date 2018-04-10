#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm_log.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Advances the cursor of a drake::lcm::DrakeLcmLog based on the timestamps
 * seen the Context that is used to simulate this System.
 *
 * This is useful when a simulated Diagram contains LcmSubscriberSystem(s)
 * whose outputs should be determined by logged data and when the log's cursor
 * should advance automatically during simulation.
 */
class LcmLogPlaybackSystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmLogPlaybackSystem)

  /**
   * Constructs a playback system that advances the given @p log.
   *
   * @param log non-null pointer that is aliased and retained by this object.
   */
  explicit LcmLogPlaybackSystem(drake::lcm::DrakeLcmLog* log);

  ~LcmLogPlaybackSystem() override;

 protected:
  void DoCalcNextUpdateTime(const Context<double>&,
                            systems::CompositeEventCollection<double>*,
                            double*) const override;

 private:
  drake::lcm::DrakeLcmLog* const log_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
