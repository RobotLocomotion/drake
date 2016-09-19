#pragma once

#include <cstdint>
#include <vector>

#include "drake/drake_rbp_export.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/systems/framework/vector_base.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {
namespace systems {

/**
 * Specializes `LcmAndVectorBaseTranslator` to handle LCM messages of type
 * `drake::lcmt_viewer_draw`. It translates between a VectorBase<double> that
 * contains the generalized state of a RigidBodyTree, and a
 * `drake::lcmt_viewer_draw` message.
 */
class DRAKE_RBP_EXPORT ViewerDrawTranslator
    : public lcm::LcmAndVectorBaseTranslator {
 public:
  // Defines the number of states per body. There are seven states: three
  // position states specified by a 3-vector (x, y, and z) and four orientation
  // states specified by a quaternion 4-vector (w, x, y, z).
  static const int kNumStatesPerBody = 7;

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
  explicit ViewerDrawTranslator(const RigidBodyTree& tree);

  void Deserialize(
      const void* lcm_message_bytes, int lcm_message_length,
      VectorBase<double>* vector_base) const override;

  void Serialize(double time,
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
