#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/physical_frame.h"

namespace drake {
namespace multibody {

// Forward declarations.
template <class T> class MultibodyTree;
template <class T> class RigidBody;

/// This class represents a frame `F` with pose `X_BF` measured and expressed in
/// the body frame `B` of a rigid body.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class RigidBodyFrame : public PhysicalFrame<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RigidBodyFrame)

  /// Creates a new %RigidBodyFrame and adds it to the MultibodyTree `tree`.
  /// The input MultibodyTree takes ownership of the newly created frame.
  ///
  /// @note This method invalidates the topology of the MultibodyTree `tree`.
  /// Users must call the Compile() method on `tree` in order to re-compute and
  /// validate its topology. See the documentation on Compile() for details.
  ///
  /// @param[in, out] tree The parent MultibodyTree to which this frame will be
  ///                      added.
  /// @returns A constant reference to the newly created rigid body frame.
  static RigidBodyFrame<T>& Create(
      MultibodyTree<T>* tree,
      const RigidBody<T>& body, const Isometry3<T>& X_BM);

  void Compile() final {}

 private:
  // Users are forced to create new rigid body frames with the Create() factory.
  RigidBodyFrame(const RigidBody<T>& B, const Isometry3<T>& X_BM);

  Isometry3<T> X_BM_;
};

}  // namespace multibody
}  // namespace drake
