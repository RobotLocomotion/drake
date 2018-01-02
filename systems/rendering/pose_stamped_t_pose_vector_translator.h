#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"

namespace drake {
namespace systems {
namespace rendering {

/// The translator that converts from/to rendering::PoseVector<double> to/from
/// lcm message bytes which encodes robotlocomotion::pose_stamped_t.
class PoseStampedTPoseVectorTranslator
    : public lcm::LcmAndVectorBaseTranslator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PoseStampedTPoseVectorTranslator)

  explicit PoseStampedTPoseVectorTranslator(const std::string& frame_name);

  void Deserialize(const void* lcm_message_bytes, int lcm_message_length,
                   VectorBase<double>* vector_base) const  override;

  void Serialize(double time, const VectorBase<double>& vector_base,
                 std::vector<uint8_t>* lcm_message_bytes) const override;

  std::unique_ptr<BasicVector<double>> AllocateOutputVector() const override;

 private:
  const std::string frame_name_;
};

}  // namespace rendering
}  // namespace systems
}  // namespace drake
