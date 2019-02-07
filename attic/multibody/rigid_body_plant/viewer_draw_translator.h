#pragma once

#include <cstdint>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/**
 * Translates between a VectorBase<double> that contains the position or state
 * vector of a RigidBodyTree, and a `drake::lcmt_viewer_draw` message.
 */
class ViewerDrawTranslator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ViewerDrawTranslator)

  /**
   * A constructor that sets the expected size of both the LCM message and
   * VectorBase vector to be the size of the position vector of @p tree.
   * However, the Serialize() method will also accept input the size of the
   * state vector of @p tree, which is the sum of the number of position and
   * velocity states in @p tree.
   *
   * @param[in] tree A reference to the RigidBodyTree with which to obtain the
   * pose of each RigidBody to be included in the `drake::lcmt_viewer_draw` LCM
   * message. This reference must remain valid for the lifetime of this object.
   */
  explicit ViewerDrawTranslator(const RigidBodyTree<double>& tree);

  /** Returns the size of the VectorBase */
  int get_vector_size() const {
    return tree_.get_num_positions();
  }

  /**
   * <b>This method must not be called.</b> It is not implemented and will abort
   * if called. The method <i>could</i> be implement should the need arise.
   */
  void Deserialize(
      const void* lcm_message_bytes, int lcm_message_length,
      VectorBase<double>* vector_base) const;

  void Serialize(double time,
      const VectorBase<double>& vector_base,
      std::vector<uint8_t>* lcm_message_bytes) const;

 private:
  // The RigidBodyTree with which the poses of each RigidBody can be
  // determined given the state vector of the RigidBodyTree.
  const RigidBodyTree<double>& tree_;

  // A partially initialized LCM draw message. This is used by
  // ViewerDrawTranslator::Serialize() to avoid having to re-initialize the
  // message each time it publishes draw message.
  drake::lcmt_viewer_draw draw_message_;
};

}  // namespace systems
}  // namespace drake
