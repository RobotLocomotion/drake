#pragma once

#include <cstdint>
#include <vector>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Specializes `LcmAndVectorBaseTranslator` to handle LCM messages of type
 * `drake::lcmt_viewer_draw`.
 */
class DRAKELCMSYSTEM2_EXPORT TranslatorBetweenVectorBaseAndLcmtViewerDraw
    : public LcmAndVectorBaseTranslator {
 public:
  /**
   * The constructor.
   *
   * @param[in] tree A reference to the RigidBodyTree with which to obtain the
   * pose of each RigidBody to be included in the `drake::lcmt_viewer_draw` LCM
   * message. This reference must remain valid for the lifetime of the object
   * instantiation of this class.
   *
   * @param[in] size The number of elements in the `drake::systems::VectorBase`
   * that is to be translated. It is assumed that the LCM message
   * `drake::lcmt_drake_signal` has the same number of elements.
   */
  explicit TranslatorBetweenVectorBaseAndLcmtViewerDraw(
      const RigidBodyTree& tree, int size)
      : LcmAndVectorBaseTranslator(size) {}

  void TranslateLcmToVectorBase(
      const void* lcm_message_bytes, int lcm_message_length,
      VectorBase<double>* vector_base) const override;

  void TranslateVectorBaseToLcm(
      const VectorBase<double>& vector_base,
      std::vector<uint8_t>* lcm_message_bytes) const override;

  private:
    // The RigidBodyTree with which the poses of each RigidBody can be
    // determined given the generalized state of the RigidBodyTree.
    RigidBodyTree& tree_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
