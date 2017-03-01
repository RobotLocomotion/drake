#pragma once

#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/math/spatial_vector.h"

namespace drake {
namespace multibody {

// forward declaration for the traits struct below.
template <typename T> class SpatialForce;
namespace internal {
// traits specialization for SpatialForce.
template <typename T>
struct spatial_vector_traits<SpatialForce<T>> {
typedef T ScalarType;
};
}

/// This class is used to represent physical quantities that correspond to
/// spatial forces. spatial forces are 6-element quantities that are
/// pairs of ordinary 3-vectors. Elements 0-2 are always the angular velocity
/// while elements 3-5 are the linear velocity.
/// For a more detailed introduction on spatial vectors please refer to
/// section @ref multibody_spatial_vectors.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
template <typename T>
class SpatialForce : public SpatialVector<SpatialForce<T>> {
  typedef SpatialVector<SpatialForce<T>> Base;
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpatialForce)

  using Base::translational;
  using Base::rotational;

  /// Default constructor. In Release builds the elements of the newly
  /// constructed spatial force are left uninitialized resulting in a zero
  /// cost operation. However in Debug builds those entries are set to NaN so
  /// that operations using this uninitialized spatial force fail fast,
  /// allowing fast bug detection.
  SpatialForce() : Base() {}

  /// SpatialForce constructor from a torque @p τ and a force @p f.
  SpatialForce(const Eigen::Ref<const Vector3<T>>& w,
               const Eigen::Ref<const Vector3<T>>& v) : Base(w, v) {}

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
    rotational() -= p_AB_E.cross(translational());
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

  /// Multiplication of a spatial force `V` from the left by a scalar `s`.
  /// @relates SpatialForce.
  friend SpatialForce<T> operator*(const T& s, const SpatialForce<T>& V) {
    return SpatialForce<T>(s * V.get_coeffs());
  }

  /// Multiplication of a spatial force `V` from the right by a scalar `s`.
  /// @relates SpatialForce.
  friend SpatialForce<T> operator*(const SpatialForce<T>& V, const T& s) {
    return s * V;  // Multiplication by scalar is commutative.
  }

  /// Returns the dot-product between `this` spatial force and the spatial
  /// velocity @p V. Both spatial quantities must be expressed in the same frame.
  T dot(const SpatialVelocity<T>& V) const;
};

}  // namespace multibody
}  // namespace drake
