#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <vector>

#include "drake/automotive/gen/single_lane_ego_and_agent_state.h"
#include "drake/common/drake_export.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"
//#include "drake/lcmt_single_lane_ego_and_agent_state_t.hpp"

namespace drake {
namespace automotive {

/**
 * Translates between LCM message objects and VectorBase objects for the
 * SingleLaneEgoAndAgentState type.
 */
class DRAKE_EXPORT SingleLaneEgoAndAgentStateTranslator
    : public systems::lcm::LcmAndVectorBaseTranslator {
 public:
  SingleLaneEgoAndAgentStateTranslator()
      : LcmAndVectorBaseTranslator(
                        SingleLaneEgoAndAgentStateIndices::kNumCoordinates) {}
  std::unique_ptr<systems::BasicVector<double>> AllocateOutputVector()
      const override;
  void Deserialize(const void* lcm_message_bytes, int lcm_message_length,
                   systems::VectorBase<double>* vector_base) const override;
  void Serialize(double time, const systems::VectorBase<double>& vector_base,
                 std::vector<uint8_t>* lcm_message_bytes) const override;
};

}  // namespace automotive
}  // namespace drake
