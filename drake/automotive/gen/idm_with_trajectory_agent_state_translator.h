#pragma once

// This file is generated by a script.  Do not edit!
// See drake/automotive/lcm_vector_gen.py.

#include "drake/automotive/gen/idm_with_trajectory_agent_state.h"
#include "drake/drakeAutomotive_export.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"
#include "lcmtypes/drake/lcmt_idm_with_trajectory_agent_state_t.hpp"

namespace drake {
namespace automotive {

/**
 * Translates between LCM message objects and VectorBase objects for the
 * IdmWithTrajectoryAgentState type.
 */
class DRAKEAUTOMOTIVE_EXPORT IdmWithTrajectoryAgentStateTranslator
    : public systems::lcm::LcmAndVectorBaseTranslator {
 public:
  IdmWithTrajectoryAgentStateTranslator()
      : LcmAndVectorBaseTranslator(
            IdmWithTrajectoryAgentStateIndices::kNumCoordinates) {}
  std::unique_ptr<systems::BasicVector<double>> AllocateOutputVector()
      const override;
  void TranslateLcmToVectorBase(
      const void* lcm_message_bytes, int lcm_message_length,
      systems::VectorBase<double>* vector_base) const override;
  void TranslateVectorBaseToLcm(
      const systems::VectorBase<double>& vector_base, double time,
      std::vector<uint8_t>* lcm_message_bytes) const override;
};

}  // namespace automotive
}  // namespace drake
