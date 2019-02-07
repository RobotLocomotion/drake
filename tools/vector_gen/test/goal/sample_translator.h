#pragma once

// GENERATED GOAL DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <memory>
#include <vector>

#include "drake/common/drake_deprecated.h"
#include "drake/lcmt_sample_t.hpp"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"
#include "drake/tools/vector_gen/test/gen/sample.h"

namespace drake {
namespace tools {
namespace test {

/**
 * Translates between LCM message objects and VectorBase objects for the
 * Sample type.
 */
class DRAKE_DEPRECATED(
    "The LcmAndVectorBaseTranslator and its related code are deprecated, "
    "and will be removed on 2019-05-01.") SampleTranslator final
    : public drake::systems::lcm::LcmAndVectorBaseTranslator {
 public:
  SampleTranslator()
      : LcmAndVectorBaseTranslator(SampleIndices::kNumCoordinates) {}
  std::unique_ptr<drake::systems::BasicVector<double>> AllocateOutputVector()
      const final;
  void Deserialize(const void* lcm_message_bytes, int lcm_message_length,
                   drake::systems::VectorBase<double>* vector_base) const final;
  void Serialize(double time,
                 const drake::systems::VectorBase<double>& vector_base,
                 std::vector<uint8_t>* lcm_message_bytes) const final;
};

}  // namespace test
}  // namespace tools
}  // namespace drake
