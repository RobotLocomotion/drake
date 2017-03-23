#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/physical_frame.h"

namespace drake {
namespace multibody {

// Forward declarations.
template <class T> class MultibodyTree;
template <class T> class RigidBody;

/// This class represents a frame `F` with a fixed pose `X_PF` measured and
/// expressed in another physical frame `P`. For instance, we could rigidly
/// attach a frame `F` to move with a rigid body `B` at a fixed pose `X_BF`
/// measured and expressed in the frame of that body.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class FixedOffsetFrame : public PhysicalFrame<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FixedOffsetFrame)

  /// Creates a physical frame with a fixed pose `X_BF` measured and expressed
  /// in the frame `B` of the input `body`.
  /// The new %FixedOffsetFrame is added to the MultibodyTree `tree`, which
  /// takes ownership of the newly created frame.
  ///
  /// @note This method invalidates the topology of the MultibodyTree `tree`.
  /// Users must call the Compile() method on `tree` in order to re-compute and
  /// validate its topology. See the documentation on Compile() for details.
  ///
  /// @param[in, out] tree The parent MultibodyTree to which this frame will be
  ///                      added.
  /// @param[in] body The body to which this frame is attached to with a fixed
  ///                 pose to it.
  /// @param[in] X_BF The fixed pose of the newly created frame in the frame `B`
  ///                 of `body`.
  /// @returns A constant reference to the newly created frame.
  static FixedOffsetFrame<T>& Create(
      MultibodyTree<T>* tree,
      const Body<T>& body, const Isometry3<T>& X_BF);

  /// Creates a physical frame with a fixed pose `X_PF` measured and expressed
  /// in another physical frame `P`.
  /// The new %FixedOffsetFrame is added to the MultibodyTree `tree`, which
  /// takes ownership of the newly created frame.
  ///
  /// @note This method invalidates the topology of the MultibodyTree `tree`.
  /// Users must call the Compile() method on `tree` in order to re-compute and
  /// validate its topology. See the documentation on Compile() for details.
  ///
  /// @warning This factory will allow, in future implementations, to chain
  /// multiple frames of type FixedOffsetFrame. However, this chaining is not
  /// yet supported and attempts to fix a %FixedOffsetFrame to a frame with type
  /// other than BodyFrame will throw an exception of type std::logic_error.
  ///
  /// @param[in, out] tree The parent MultibodyTree to which this frame will be
  ///                      added.
  /// @param[in] P The physical frame to which this frame is attached to with a
  ///              fixed pose to it.
  /// @param[in] X_PF The fixed pose of the newly created frame measured and
  ///                 expressed in the physical frame `P`.
  /// @returns A constant reference to the newly created frame.
  // TODO(amcastro-tri): allow to chain multiple frames of type
  // FixedOffsetFrame. An approach would consist on holding a reference to the
  // parent frame of the root FixedOffsetFrame of the chain and X_PF_ would
  // then be set to (at construction) to the pose of this frame on that parent
  // frame.
  static FixedOffsetFrame<T>& Create(
      MultibodyTree<T>* tree,
      const PhysicalFrame<T>& P, const Isometry3<T>& X_PF);

  void Compile() final {}

 private:
  // Users are forced to create new rigid body frames with the Create()
  // factories.
  FixedOffsetFrame(const PhysicalFrame<T>& P, const Isometry3<T>& X_PF);

  // The fixed pose of this frame F measured and expressed in another physical
  // frame P.
  Isometry3<T> X_PF_;
};

}  // namespace multibody
}  // namespace drake
