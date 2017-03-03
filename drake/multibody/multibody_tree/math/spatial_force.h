#pragma once

#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/math/spatial_vector.h"

namespace drake {
namespace multibody {

// Forward declaration to define dot product with a spatial velocity.
template <typename T> class SpatialVelocity;

/// This class is used to represent spatial forces. Spatial forces are 6-element
/// quantities that are pairs of ordinary 3-vectors. Elements 0-2 are always the
/// torque component while elements 3-5 are the force component.
/// The translational force is understood to be applied at a specific point but
/// that point is not stored within the SpatialForce class. Both the
/// translational force and torque are expected to be expressed in the same
/// frame however, this class does not offer any mechanism to track the
/// expresseed-in frame. It is the responsibility of the user to keep track the
/// point at which the spatial force is applied and the expressed-in frame. In
/// source code the monogram notation `F_Po_E` is used to represent the spatial
/// force `F` at a point `Po` expressed in frame `E`.
/// For a more detailed introduction on spatial vectors and the monogram
/// notation please refer to section @ref multibody_spatial_vectors.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
template <typename T>
class SpatialForce : public SpatialVector<SpatialForce, T> {
  // We need the fully qualified class name below for the clang compiler to
  // work. Without qualifiers the code is legal according to the C++11 standard
  // but the clang compiler still gets confused. See:
  // http://stackoverflow.com/questions/17687459/clang-not-accepting-use-of-template-template-parameter-when-using-crtp
  typedef SpatialVector<::drake::multibody::SpatialForce, T> Base;

 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpatialForce)

  /// Default constructor. In Release builds the elements of the newly
  /// constructed spatial force are left uninitialized resulting in a zero
  /// cost operation. However in Debug builds those entries are set to NaN so
  /// that operations using this uninitialized spatial force fail fast,
  /// allowing fast bug detection.
  SpatialForce() : Base() {}

  /// SpatialForce constructor from a torque `tau` and a force `f`.
  SpatialForce(const Eigen::Ref<const Vector3<T>>& tau,
               const Eigen::Ref<const Vector3<T>>& f) : Base(tau, f) {}

  /// SpatialForce constructor from an Eigen expression that represents a
  /// six-dimensional vector.
  /// This constructor will assert the size of `V` is six (6) at compile-time
  /// for fixed sized Eigen expressions and at run-time for dynamic sized Eigen
  /// expressions.
  template <typename Derived>
  explicit SpatialForce(const Eigen::MatrixBase<Derived>& V) : Base(V) {}

  /// Given `this` spatial force `F_Ao_E` applied at the origin of a frame `A`
  /// and expressed in a frame `E`, this method computes the spatial force
  /// applied at the origin `Bo` of another frame `B` rigidly moving with `A`
  /// but offset by a vector `p_AB_E` from the orgin of frame `Ao` to the origin
  /// of frame `B` and expressed in the same frame `E`.
  ///
  /// The operation performed, in coordinate-free form, is: <pre>
  ///   τ_Bo = τ_Ao -  p_AB x f_Ao
  ///   f_Bo = f_Ao,  i.e. the force as applied on B equals the force
  ///                 as applied on A.
  /// </pre>
  /// where τ and f represent the torque and force components respectively.
  ///
  /// All quantities above must be expressed in a common frame `E` i.e: <pre>
  ///   τ_Bo_E = τ_Ao_E -  p_AB_E x f_Ao_E
  ///   f_Bo_E = f_Ao_E
  /// </pre>
  ///
  /// This operation is performed in-place modifying the original object.
  ///
  /// @param[in] p_AB_E Shift vector from `Ao` to `Bo` and expressed in
  ///                   frame `E`.
  /// @returns A reference to `this` spatial force `F_Bo_E` now as applied about
  ///          `Bo` expressed in frame `A`.
  ///
  /// @see Shift() to compute the shifted spatial force without modifying
  ///              this original object.
  SpatialForce<T>& ShiftInPlace(const Vector3<T>& p_AB_E) {
    this->rotational() -= p_AB_E.cross(this->translational());
    return *this;
  }

  /// Given `this` spatial force `F_Ao_E` applied at the origin of a frame `A`
  /// and expressed in a frame `E`, this method computes the spatial force
  /// applied at the origin `Bo` of another frame `B` rigidly moving with `A`
  /// but offset by a vector `p_AB_E` from the orgin of frame `Ao` to the origin
  /// of frame `B` and expressed in the same frame `E`.
  ///
  /// @param[in] p_AB_E Shift vector from `Ao` to `Bo` and expressed in
  ///                   frame `E`.
  /// @retval F_Bo_E The spatial force as applied about `Bo` expressed in
  ///                frame `A`.
  ///
  /// @see ShiftInPlace() to compute the shifted spatial force in-place
  ///                     modifying the original object.
  SpatialForce<T> Shift(const Vector3<T>& p_AB_E) const {
    return SpatialForce<T>(*this).ShiftInPlace(p_AB_E);
  }

  /// Given `this` spatial force `F_Q_E` applied about the origin of frame `Q`
  /// and expressed in a frame `E`, this method computes the 6-dimensional dot
  /// product with the spatial velocity `V_IQ_E` of frame `Q` measured in an
  /// inertial frame `I` and expressed in the same frame `E` in which the
  /// spatial force is expressed.
  /// This dot-product represents the power generated by `this` spatial force
  /// `F_Q_E` when applied about the origin of frame `Q` moving with spatial
  /// velocity `V_IQ_E`.
  /// Both spatial quantities must be expressed in the same frame `E` with the
  /// result being independent of this frame `E` in which they are expressed.
  T dot(const SpatialVelocity<T>& V_IQ_E) const;
};

}  // namespace multibody
}  // namespace drake
