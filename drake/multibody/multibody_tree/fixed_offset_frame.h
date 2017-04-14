#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

// Forward declarations.
template <class T> class BodyFrame;
template <class T> class MultibodyTree;
template <class T> class RigidBody;

/// %FixedOffsetFrame represents a material Frame F whose pose is fixed with
/// respect to a _parent_ material frame P. The pose offset is given by a
/// spatial transform `X_PF`, which is constant after construction. For
/// instance, we could rigidly attach a frame F to move with a rigid body B at a
/// fixed pose `X_BF`, where B is the BodyFrame associated with body B.
/// Thus, the World frame pose `X_WF` of a %FixedOffsetFrame F depends only on
/// the World frame pose `X_WP` of its parent P, and the constant pose `X_PF`.
///
/// For more information about spatial transforms, see
/// @ref multibody_spatial_pose. <!-- http://drake.mit.edu/doxygen_cxx/
///                                   group__multibody__spatial__pose.html -->
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class FixedOffsetFrame : public Frame<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FixedOffsetFrame)

  /// Creates a material Frame F whose pose is fixed with respect to its
  /// parent material Frame P. The pose is given by a spatial transform `X_PF`;
  /// see class documentation for more information.
  ///
  /// @param[in] P The frame to which this frame is attached with a fixed pose.
  /// @param[in] X_PF The transform giving the pose of F in P.

  // TODO(amcastro-tri): allow to chain multiple frames of type
  // FixedOffsetFrame. An approach would consist on holding a reference to the
  // parent frame of the root FixedOffsetFrame of the chain and X_PF_ would
  // then be set to (at construction) to the pose of this frame on that parent
  // frame.
  FixedOffsetFrame(const BodyFrame<T>& P, const Isometry3<T>& X_PF);

  /// Creates a material Frame F whose pose is fixed with respect to the
  /// BodyFrame B of the given Body, which serves as F's parent frame.
  /// The pose is given by a spatial transform `X_BF`; see class documentation
  /// for more information.
  ///
  /// @param[in] bodyB The body whose BodyFrame B is to be F's parent frame.
  /// @param[in] X_BF  The transform giving the pose of F in B.
  FixedOffsetFrame(const Body<T>& bodyB, const Isometry3<T>& X_BF);

 private:
  // Spatial transform giving the fixed pose of this frame F in another frame P.
  Isometry3<T> X_PF_;
};

}  // namespace multibody
}  // namespace drake
