#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/mobilizer_impl.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

// Forward declarations.
template <typename T> class MultibodyTree;

namespace drake {
namespace multibody {

/// This Mobilizer grants a single degree of freedom describing the angular
/// rotation between the inboard and outboard frames it connects.
/// To fully specify this mobilizer a user must provide the inboard frame F,
/// the outbourd (or "mobilized") frame M and the axis `axis_F` (expressed in
/// frame F) about which frame M rotates with respect to F.
/// The single generalized coordinate q introduced by this mobilizer
/// corresponds to the rotation angle in radians of frame M with respect to
/// frame F about the rotation axis `axis_F`.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template <typename T>
class RevoluteMobilizer : public MobilizerImpl<T, 1, 1> {
 public:
  /// Constructor for a %RevoluteMobilizer between frames the inboard frame F
  /// `inboard_frame` and the outboard frame M `outboard_frame` granting a
  /// single rotational degree of fredom about axis `axis_F` expressed in the
  /// inboard frame F.
  RevoluteMobilizer(const Frame<T>& inboard_frame,
                    const Frame<T>& outboard_frame,
                    const Vector3<double> axis_F) :
      MobilizerBase(inboard_frame, outboard_frame), axis_F_(axis_F) {}

  /// Returns a constant reference to the axis, expressed in the inboard frame,
  /// about which the outboard frame rotates with respect to the inboard frame.
  const Vector3<double>& get_revolute_axis() const { return axis_F_; }

 private:
  typedef MobilizerImpl<T, 1, 1> MobilizerBase;
  using MobilizerBase::nq;
  using MobilizerBase::nv;

  // Default joint axis expressed in the inboard frame F.
  Vector3<double> axis_F_;
};

}  // namespace multibody
}  // namespace drake
