#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

/// Base class for specific Mobilizer implementations with the number of
/// generalized positions and velocities resolved at compile time as template
/// parameters. This allows specific mobilizer implementations to only work on
/// fixed-size Eigen expressions therefore allowing for optimized operations on
/// fixed-size matrices. In addition, this layer discourages the proliferation
/// of dynamic-sized Eigen matrices that would otherwise lead to run-time
/// dynamic memory allocations.
/// %MobilizerImpl also provides a number of size specific methods to retrieve
/// multibody quantities of interest from caching structures. These are common
/// to all mobilizer implementations and therefore they live in this class.
/// Users should not need to interact with this class directly unless they need
/// to implement a custom Mobilizer class.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T, int num_positions, int num_velocities>
class MobilizerImpl : public Mobilizer<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MobilizerImpl)

  /// As with Mobilizer this the only constructor available for this base class.
  /// The minimum amount of information that we need to define a mobilizer is
  /// the knowledge of the inboard and outboard frames it connects.
  /// Subclasses of %MobilizerImpl are therefore forced to provide this
  /// information in their respective constructors.
  MobilizerImpl(const Frame<T>& inboard_frame,
                const Frame<T>& outboard_frame) :
      Mobilizer<T>(inboard_frame, outboard_frame) {}

  /// Returns the number of generalized coordinates granted by this mobilizer.
  int get_num_positions() const final { return nq;}

  /// Returns the number of generalized velocities granted by this mobilizer.
  int get_num_velocities() const final { return nv;}

  /// Default implementation to Mobilizer::set_zero_configuration() that sets
  /// all generalized positions related to this mobilizer to zero.
  /// Be aware however that this default does not apply in general to all
  /// mobilizers and specific subclases must override this method for
  /// correctness.
  void set_zero_configuration(systems::Context<T>* context) const override {
    auto mbt_context = dynamic_cast<MultibodyTreeContext<T>*>(context);
    DRAKE_DEMAND(mbt_context != nullptr);
    get_mutable_positions(mbt_context).setZero();
  }

  /// For MultibodyTree internal use only.
  std::unique_ptr<BodyNode<T>> CreateBodyNode(
    const Body<T>& body, const Mobilizer<T>* mobilizer) const final;

 protected:
  // Handy enum to grant specific implementations compile time sizes.
  // static constexpr int i = 42; discouraged.
  // See answer in: http://stackoverflow.com/questions/37259807/static-constexpr-int-vs-old-fashioned-enum-when-and-why
  enum : int {nq = num_positions, nv = num_velocities};

  /// @name Helper Methods to Retrieve Entries from MultibodyTreeContext.

  /// Helper to return a const fixed-size Eigen::VectorBlock referencing the
  /// segment in the state vector corresponding to `this` mobilizer's state.
  Eigen::VectorBlock<const VectorX<T>, nq> get_positions(
      const MultibodyTreeContext<T>& context) const {
    return context.template get_state_segment<nq>(
        this->get_positions_start());
  }

  /// Helper to return a mutable fixed-size Eigen::VectorBlock referencing the
  /// segment in the state vector corresponding to `this` mobilizer's state.
  Eigen::VectorBlock<VectorX<T>, nq> get_mutable_positions(
      MultibodyTreeContext<T>* context) const {
    return context->template get_mutable_state_segment<nq>(
        this->get_positions_start());
  }
  /// @}

  /// @name Helper Methods to Retrieve Entries from PositionKinematicsCache.
  /// @{

  /// Returns a mutable reference to the pose `X_FM` of the outboard frame M as
  /// measured and expressed in the inboard frame F.
  Isometry3<T>& get_mutable_X_FM(PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_X_FM(this->get_topology().body_node);
  }
  /// @}

 private:
  // Returns the first entry in the global array of generalized coordinates in
  // the MultibodyTree model.
  int get_positions_start() const {
    return this->get_topology().positions_start;
  }
};

}  // namespace multibody
}  // namespace drake
