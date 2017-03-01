#pragma once

#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/math/spatial_vector.h"

namespace drake {
namespace multibody {

// forward declaration for the traits struct below.
template <typename T> class SpatialVelocity;
namespace internal {
// traits specialization for SpatialVelocity.
template <typename T>
struct spatial_vector_traits<SpatialVelocity<T>> {
typedef T ScalarType;
};
}

// Forward declaration to define dot product with a spatial force.
template <typename T> class SpatialForce;

/// This class is used to represent physical quantities that correspond to
/// spatial velocities. Spatial velocities are 6-element quantities that are
/// pairs of ordinary 3-vectors. Elements 0-2 are always the angular velocity
/// while elements 3-5 are the linear velocity.
/// For a more detailed introduction on spatial vectors please refer to
/// section @ref multibody_spatial_vectors.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
template <typename T>
class SpatialVelocity : public SpatialVector<SpatialVelocity<T>> {
  typedef SpatialVector<SpatialVelocity<T>> Base;

 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpatialVelocity)

  using Base::translational;
  using Base::rotational;

  /// Default constructor. In Release builds the elements of the newly
  /// constructed spatial velocity are left uninitialized resulting in a zero
  /// cost operation. However in Debug builds those entries are set to NaN so
  /// that operations using this uninitialized spatial velocity fail fast,
  /// allowing fast bug detection.
  SpatialVelocity() : Base() {}

  /// SpatialVelocity constructor from an angular velocity @p w and a linear
  /// velocity @p v.
  SpatialVelocity(const Eigen::Ref<const Vector3<T>>& w,
                  const Eigen::Ref<const Vector3<T>>& v) : Base(w, v) {}

  /// SpatialVelocity constructor from an Eigen expression that represents a
  /// six-dimensional vector.
  /// This constructor will assert the size of `V` is six (6) at compile-time
  /// for fixed sized Eigen expressions and at run-time for dynamic sized Eigen
  /// expressions.
  template <typename Derived>
  explicit SpatialVelocity(const Eigen::MatrixBase<Derived>& V) : Base(V) {}

  /// Given `this` spatial velocity `V_AB_E` of a frame `B` measured in a frame
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
  /// @returns A reference to `this` spatial velocity `V_AQ_E` now of frame `Q`
  ///          measured in frame `A` and expressed in frame `E`.
  ///
  /// @see Shift() to compute the shifted spatial velocity without modifying
  ///      this original object.
  SpatialVelocity<T>& ShiftInPlace(const Vector3<T>& p_BQ_E) {
    translational() += rotational().cross(p_BQ_E);
    return *this;
  }

  /// Given `this` spatial velocity `V_AB_E` of a frame `B` measured in a frame
  /// `A` and expressed in a frame `E`, this method computes the spatial
  /// velocity of a frame `Q` rigidly moving with `B` but offset by a vector
  /// `p_BQ` from the orgin of frame `B` to the origin of frame Q.
  ///
  /// @param[in] p_BQ_E Shift vector from `Bo` to `Qo` and expressed in
  ///                   frame `E`.
  /// @retval V_AQ_E The spatial velocity of frame `Q` measured in frame `A` and
  ///                expressed in frame `E`.
  ///
  /// @see ShiftInPlace() to compute the shifted spatial velocity in-place
  ///      modifying the original object.
  SpatialVelocity<T> Shift(const Vector3<T>& p_BQ_E) const {
    return SpatialVelocity<T>(*this).ShiftInPlace(p_BQ_E);
  }

  /// Multiplication of a spatial velocity `V` from the left by a scalar `s`.
  /// @relates SpatialVelocity.
  friend SpatialVelocity<T> operator*(const T& s, const SpatialVelocity<T>& V) {
    return SpatialVelocity<T>(s * V.get_coeffs());
  }

  /// Multiplication of a spatial velocity `V` from the right by a scalar `s`.
  /// @relates SpatialVelocity.
  friend SpatialVelocity<T> operator*(const SpatialVelocity<T>& V, const T& s) {
    return s * V;  // Multiplication by scalar is commutative.
  }

  /// Returns the dot-product between `this` spatial velocity and the spatial
  /// force @p F. Both spatial quantities must be expressed in the same frame.
  T dot(const SpatialForce<T>& F) const;
};

}  // namespace multibody
}  // namespace drake
