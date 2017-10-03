#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm_log.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace lcm {

class LcmLogPlaybackSystem final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmLogPlaybackSystem)
  explicit LcmLogPlaybackSystem(drake::lcm::DrakeLcmLog* log);

 private:
  void DoCalcNextUpdateTime(const Context<double>& context,
                            systems::CompositeEventCollection<double>* events,
                            double* time) const override;

  drake::lcm::DrakeLcmLog* log_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
