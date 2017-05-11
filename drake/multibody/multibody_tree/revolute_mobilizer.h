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
/// the outboard (or "mobilized") frame M and the axis `axis_F` (expressed in
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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RevoluteMobilizer)

  /// Constructor for a %RevoluteMobilizer between the inboard frame F
  /// `inboard_frame` and the outboard frame M `outboard_frame` granting a
  /// single rotational degree of freedom about axis `axis_F` expressed in the
  /// inboard frame F.
  /// @pre axis_F must be a unit vector within at least 1.0e-6. This rather
  /// loose tolerance (at least for simulation) allows users to provide "near
  /// unity" axis vectors originated, for instance, during the parsing of a
  /// file with limited precision. Internally, we re-normalize the axis to
  /// within machine precision.
  /// @throws std::runtime_error if the provided rotational axis is not a unit
  /// vector.
  RevoluteMobilizer(const Frame<T>& inboard_frame,
                    const Frame<T>& outboard_frame,
                    const Vector3<double>& axis_F) :
      MobilizerBase(inboard_frame, outboard_frame), axis_F_(axis_F) {
    if (!axis_F.isUnitary(1.0e-6)) {
      throw std::runtime_error("The rotation axis must be a unit vector");
    }
    // We allow a rather loose tolerance in the isUnitary check above so that we
    // don't get spurious exceptions when, for instance, the provided axis comes
    // from parsing a file. Therefore we re-normalize here.
    axis_F_.normalize();
  }

  /// Returns a constant reference to the axis, expressed in the inboard frame,
  /// about which the outboard frame rotates with respect to the inboard frame.
  const Vector3<double>& get_revolute_axis() const { return axis_F_; }

 private:
  typedef MobilizerImpl<T, 1, 1> MobilizerBase;
  // Bring the handy number of position and velocities MobilizerImpl enums into
  // this class' scope. This is useful when writing mathematical expressions
  // with fixed-sized vectors since we can do things like Vector<T, nq>.
  // Operations with fixed-sized quantities can be optimized at compile time
  // and therefore they are highly preferred compared to the very slow dynamic
  // sized quantities.
  using MobilizerBase::nq;
  using MobilizerBase::nv;

  // Default joint axis expressed in the inboard frame F.
  Vector3<double> axis_F_;
};

}  // namespace multibody
}  // namespace drake
