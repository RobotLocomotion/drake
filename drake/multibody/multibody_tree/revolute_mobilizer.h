#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/mobilizer_impl.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

/// This Mobilizer allows two frames to rotate relatively to one another around
/// an axis that is constant when measured in either this mobilizer's inboard or
/// outboard frames, while the distance between the two frames does not vary.
/// To fully specify this mobilizer a user must provide the inboard frame F,
/// the outboard (or "mobilized") frame M and the axis `axis_F` (expressed in
/// frame F) about which frame M rotates with respect to F.
/// The single generalized coordinate q introduced by this mobilizer
/// corresponds to the rotation angle in radians of frame M with respect to
/// frame F about the rotation axis `axis_F`. When `q = 0`, frames F and M are
/// coincident. The rotation angle is defined to be positive according to the
/// right-hand-rule with the thumb aligned in the direction of the `axis_F`.
/// Notice that the components of the rotation axis as expressed in
/// either frame F or M are constant. That is, `axis_F` and `axis_M` remain
/// unchanged w.r.t. both frames by this mobilizer's motion.
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
  /// `inboard_frame_F` and the outboard frame M `outboard_frame_F` granting a
  /// single rotational degree of freedom about axis `axis_F` expressed in the
  /// inboard frame F.
  /// @pre axis_F must be a unit vector within at least 1.0e-6. This rather
  /// loose tolerance (at least for simulation) allows users to provide "near
  /// unity" axis vectors originated, for instance, during the parsing of a
  /// file with limited precision. Internally, we re-normalize the axis to
  /// within machine precision.
  /// @throws std::runtime_error if the provided rotational axis is not a unit
  /// vector.
  RevoluteMobilizer(const Frame<T>& inboard_frame_F,
                    const Frame<T>& outboard_frame_M,
                    const Vector3<double>& axis_F) :
      MobilizerBase(inboard_frame_F, outboard_frame_M), axis_F_(axis_F) {
    if (!axis_F.isUnitary(1.0e-6)) {
      throw std::runtime_error("The rotation axis must be a unit vector");
    }
    // We allow a rather loose tolerance in the isUnitary check above so that we
    // don't get spurious exceptions when, for instance, the provided axis comes
    // from parsing a file. Therefore we re-normalize here.
    axis_F_.normalize();
  }

  /// @retval axis_F The rotation axis as a unit vector expressed in the inboard
  ///                frame F.
  const Vector3<double>& get_revolute_axis() const { return axis_F_; }

  /// Gets the rotation angle of `this` mobilizer from `context`. See class
  /// documentation for sign convention.
  /// @throws std::logic_error if `context` is not a valid
  /// MultibodyTreeContext.
  /// @param[in] context The context of the MultibodyTree this mobilizer
  ///                    belongs to.
  /// @returns The angle coordinate of `this` mobilizer in the `context`.
  const T& get_angle(const systems::Context<T>& context) const;

  /// Sets the `context` so that the generalized coordinate corresponding to the
  /// rotation angle of `this` mobilizer equals `angle`.
  /// @throws std::logic_error if `context` is not a valid
  /// MultibodyTreeContext.
  /// @param[in] context The context of the MultibodyTree this mobilizer
  ///                    belongs to.
  /// @param[in] angle The desired angle in radians.
  /// @returns a constant reference to `this` mobilizer.
  const RevoluteMobilizer<T>& set_angle(
      systems::Context<T>* context, const T& angle) const;

  /// Computes the across-mobilizer transform `X_FM(q)` between the inboard
  /// frame F and the outboard frame M as a function of the rotation angle
  /// about this mobilizer's axis (@see get_revolute_axis().)
  Isometry3<T> CalcAcrossMobilizerTransform(
      const MultibodyTreeContext<T>& context) const final;

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
