#pragma once

#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {

/// This class is used to represent physical quantities that correspond to
/// spatial velocities. Spatial velocities are 6-element quantities that are
/// pairs of ordinary 3-vectors. Elements 0-2 are always the angular velocity
/// while elements 3-5 are the linear velocity.
/// For a more detailed introduction on spatial vectors please refer to
/// section @ref multibody_spatial_vectors.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
template <typename T>
class SpatialVelocity {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpatialVelocity)

  /// Sizes for spatial quantities and its components in three dimensions.
  enum {
    kSpatialVelocitySize = 6,
    kRotationSize = 3,
    kTranslationSize = 3
  };
  /// The type of the underlying in-memory representation using an Eigen vector.
  typedef Vector6<T> CoeffsEigenType;

  /// Default constructor. In Release builds the elements of the newly
  /// constructed spatial velocity are left uninitialized resulting in a zero
  /// cost operation. However in Debug builds those entries are set to NaN so
  /// that operations using this uninitialized spatial velocity fail fast,
  /// allowing fast bug detection.
  SpatialVelocity() {
    DRAKE_ASSERT_VOID(SetNaN());
  }

  /// SpatialVelocity constructor from an angular velocity @p w and a linear
  /// velocity @p v.
  SpatialVelocity(const Eigen::Ref<const Vector3<T>>& w,
                  const Eigen::Ref<const Vector3<T>>& v) {
    V_.template head<3>() = w;
    V_.template tail<3>() = v;
  }

  /// SpatialVelocity constructor from an Eigen expression that represents a
  /// six-dimensional vector.
  /// This constructor will assert the size of `V` is six (6) at compile-time
  /// for fixed sized Eigen expressions and at run-time for dynamic sized Eigen
  /// expressions.
  template <typename Derived>
  explicit SpatialVelocity(const Eigen::MatrixBase<Derived>& V) : V_(V) {}

  /// The total size of the concatenation of the angular and linear components.
  /// In three dimensions this is six (6) and it is known at compile time.
  int size() const { return kSpatialVelocitySize; }

  /// Const access to the i-th component of this spatial velocity.
  /// Bounds are only checked in Debug builds for a zero overhead implementation
  /// in Release builds.
  const T& operator[](int i) const {
    DRAKE_ASSERT(0 <= i && i < kSpatialVelocitySize);
    return V_[i];
  }

  /// Mutable access to the i-th component of this spatial velocity.
  /// Bounds are only checked in Debug builds for a zero overhead implementation
  /// in Release builds.
  T& operator[](int i) {
    DRAKE_ASSERT(0 <= i && i < kSpatialVelocitySize);
    return V_[i];
  }

  /// Const access to the rotational component of this spatial velocity.
  const Vector3<T>& rotational() const {
    // We are counting on a particular representation for an Eigen Vector3<T>:
    // it must be represented exactly as 3 T's in an array with no metadata.
    return *reinterpret_cast<const Vector3<T>*>(V_.data());
  }

  /// Mutable access to the rotational component of this spatial velocity.
  Vector3<T>& rotational() {
    // We are counting on a particular representation for an Eigen Vector3<T>:
    // it must be represented exactly as 3 T's in an array with no metadata.
    return *reinterpret_cast<Vector3<T>*>(V_.data());
  }

  /// Const access to the translational component of this spatial velocity.
  const Vector3<T>& translational() const {
    // We are counting on a particular representation for an Eigen Vector3<T>:
    // it must be represented exactly as 3 T's in an array with no metadata.
    return *reinterpret_cast<const Vector3<T>*>(
        V_.data() + kRotationSize);
  }

  /// Mutable access to the translational component of this spatial velocity.
  Vector3<T>& translational() {
    // We are counting on a particular representation for an Eigen Vector3<T>:
    // it must be represented exactly as 3 T's in an array with no metadata.
    return *reinterpret_cast<Vector3<T>*>(
        V_.data() + kRotationSize);
  }

  /// Returns a (const) bare pointer to the underlying data.
  /// It is guaranteed that there will be six (6) T's densely packed at data[0],
  /// data[1], etc.
  const T* data() const { return V_.data(); }

  /// Returns a (mutable) bare pointer to the underlying data.
  /// It is guaranteed that there will be six (6) T's densely packed at data[0],
  /// data[1], etc.
  T* mutable_data() { return V_.data(); }

  /// @returns `true` if `other` is within a precision given by @p tolerance.
  /// The comparison is performed by comparing the angular (linear) component of
  /// `this` spatial velocity with the angular (linear) component of @p other
  /// using the fuzzy comparison provided by Eigen's method isApprox().
  bool IsApprox(const SpatialVelocity<T>& other,
                double tolerance = Eigen::NumTraits<T>::epsilon()) {
    return translational().isApprox(other.translational(), tolerance) &&
           rotational().isApprox(other.rotational(), tolerance);
  }

  /// Sets all entries in `this` SpatialVelocity to NaN. Typically used to
  /// quickly detect uninitialized values since NaN will trigger a chain of
  /// invalid computations that can then be tracked back to the source.
  void SetNaN() {
    V_.setConstant(std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN());
  }

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
  /// @retval V_AQ_E The spatial velocity of frame `Q` with respect to `A` and
  ///                expressed in frame `A`.
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
  /// @retval V_AQ_E The spatial velocity of frame `Q` with respect to `A` and
  ///                expressed in frame `A`.
  ///
  /// @see ShiftInPlace() to compute the shifted spatial velocity in-place
  ///      modifying the original object.
  SpatialVelocity<T> Shift(const Vector3<T>& p_BQ_E) const {
    return SpatialVelocity<T>(*this).ShiftInPlace(p_BQ_E);
  }

  /// Multiplication of a spatial velocity `V` from the left by a scalar `s`.
  /// @relates SpatialVelocity.
  friend SpatialVelocity<T> operator*(const T& s, const SpatialVelocity<T>& V) {
    return SpatialVelocity<T>(s * V.V_);
  }

  /// Multiplication of a spatial velocity `V` from the right by a scalar `s`.
  /// @relates SpatialVelocity.
  friend SpatialVelocity<T> operator*(const SpatialVelocity<T>& V, const T& s) {
    return s * V;  // Multiplication by scalar is commutative.
  }

 private:
  CoeffsEigenType V_;
};

/// Stream insertion operator to write SpatialVelocity objects into a
/// `std::ostream`. Especially useful for debugging.
/// @relates SpatialVelocity.
template <typename T> inline
std::ostream& operator<<(std::ostream& o, const SpatialVelocity<T>& V) {
  o << "[" << V[0];
  for (int i = 1; i < V.size(); ++i) o << ", " << V[i];
  o << "]ᵀ";  // The "transpose" symbol.
  return o;
}

}  // namespace multibody
}  // namespace drake
