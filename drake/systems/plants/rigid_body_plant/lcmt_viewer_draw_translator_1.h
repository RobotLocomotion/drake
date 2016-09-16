#pragma once

#include <cstdint>
#include <vector>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/systems/framework/vector_base.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {
namespace systems {

/**
 * Specializes `LcmAndVectorBaseTranslator` to handle LCM messages of type
 * `drake::lcmt_viewer_draw`.
 */
class DRAKELCMSYSTEM2_EXPORT LcmtViewerDrawTranslator1
    : public lcm::LcmAndVectorBaseTranslator {
 public:
  /**
   * A constructor that initializes the internal state of this class and sets
   * the expected sizes of both the LCM message and VectorBase vector to be the
   * size of the generalized state of the RigidBodyTree, which is the sum of the
   * position and velocity states in the RigidBodyTree.
   *
   * @param[in] tree A reference to the RigidBodyTree with which to obtain the
   * pose of each RigidBody to be included in the `drake::lcmt_viewer_draw` LCM
   * message. This reference must remain valid for the lifetime of the object
   * instantiation of this class.
   */
  explicit LcmtViewerDrawTranslator1(const RigidBodyTree& tree);

  void TranslateLcmToVectorBase(
      const void* lcm_message_bytes, int lcm_message_length,
      VectorBase<double>* vector_base) const override;

  void TranslateVectorBaseToLcm(double time,
      const VectorBase<double>& vector_base,
      std::vector<uint8_t>* lcm_message_bytes) const override;

 private:
  // Initializes member variable draw_msg_. This is only called once by the
  // constructor.
  void initialize_draw_message();

  // The RigidBodyTree with which the poses of each RigidBody can be
  // determined given the generalized state of the RigidBodyTree.
  const RigidBodyTree& tree_;

  // The LCM draw message to send to the Drake Visualizer. This member variable
  // is declared mutable so it can be modified by EvalOutput().
  drake::lcmt_viewer_draw draw_msg_;
};

}  // namespace systems
}  // namespace drake
