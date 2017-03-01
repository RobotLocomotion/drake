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
/// spatial velocities. Spatial velocities are 6-element quantities that are
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

  /// SpatialForce constructor from an angular velocity @p w and a linear
  /// velocity @p v.
  SpatialForce(const Eigen::Ref<const Vector3<T>>& w,
               const Eigen::Ref<const Vector3<T>>& v) : Base(w, v) {}

  /// SpatialForce constructor from an Eigen expression that represents a
  /// six-dimensional vector.
  /// This constructor will assert the size of `V` is six (6) at compile-time
  /// for fixed sized Eigen expressions and at run-time for dynamic sized Eigen
  /// expressions.
  template <typename Derived>
  explicit SpatialForce(const Eigen::MatrixBase<Derived>& V) : Base(V) {}

  /// Given `this` spatial force `V_AB_E` of a frame `B` measured in a frame
  /// `A` and expressed in a frame `E`, this method computes the spatial
  /// velocity of a frame `Q` rigidly moving with `B` but offset by a vector
  /// `p_BQ_E` from the orgin of frame `B` to the origin of frame `Q` and
  /// expressed in the same frame `E`.
  ///
  /// The operation performed, in coordinate-free form, is: <pre>
  ///   w_AQ = w_AB,  i.e. the angular velocity of frame B and Q is the same.
  ///   v_AQ = v_AB + w_AB x p_BQ
  /// </pre>
  ///
  /// All quantities above must be expressed in a common frame `E` i.e: <pre>
  ///   w_AQ_E = w_AB_E
  ///   v_AQ_E = v_AB_E + w_AB_E x p_BQ_E
  /// </pre>
  ///
  /// This operation is performed in-place modifying the original object.
  ///
  /// @param[in] p_BQ_E Shift vector from `Bo` to `Qo` and expressed in
  ///                   frame `E`.
  /// @retval V_AQ_E The spatial force of frame `Q` with respect to `A` and
  ///                expressed in frame `A`.
  ///
  /// @see Shift() to compute the shifted spatial force without modifying
  ///      this original object.
  SpatialForce<T>& ShiftInPlace(const Vector3<T>& p_BQ_E) {
    translational() += rotational().cross(p_BQ_E);
    return *this;
  }

  /// Given `this` spatial force `V_AB_E` of a frame `B` measured in a frame
  /// `A` and expressed in a frame `E`, this method computes the spatial
  /// velocity of a frame `Q` rigidly moving with `B` but offset by a vector
  /// `p_BQ` from the orgin of frame `B` to the origin of frame Q.
  ///
  /// @param[in] p_BQ_E Shift vector from `Bo` to `Qo` and expressed in
  ///                   frame `E`.
  /// @retval V_AQ_E The spatial force of frame `Q` with respect to `A` and
  ///                expressed in frame `A`.
  ///
  /// @see ShiftInPlace() to compute the shifted spatial force in-place
  ///      modifying the original object.
  SpatialForce<T> Shift(const Vector3<T>& p_BQ_E) const {
    return SpatialForce<T>(*this).ShiftInPlace(p_BQ_E);
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
};

}  // namespace multibody
}  // namespace drake
