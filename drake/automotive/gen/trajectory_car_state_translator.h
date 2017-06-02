#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <memory>
#include <vector>

#include "drake/automotive/gen/trajectory_car_state.h"
#include "drake/lcmt_trajectory_car_state_t.hpp"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"

namespace drake {
namespace automotive {

/**
 * Translates between LCM message objects and VectorBase objects for the
 * TrajectoryCarState type.
 */
class TrajectoryCarStateTranslator
    : public systems::lcm::LcmAndVectorBaseTranslator {
 public:
  TrajectoryCarStateTranslator()
      : LcmAndVectorBaseTranslator(TrajectoryCarStateIndices::kNumCoordinates) {
  }
  std::unique_ptr<systems::BasicVector<double>> AllocateOutputVector()
      const override;
  void Deserialize(const void* lcm_message_bytes, int lcm_message_length,
                   systems::VectorBase<double>* vector_base) const override;
  void Serialize(double time, const systems::VectorBase<double>& vector_base,
                 std::vector<uint8_t>* lcm_message_bytes) const override;
};

}  // namespace automotive
}  // namespace drake
