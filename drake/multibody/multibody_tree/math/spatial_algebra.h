#pragma once

/// @file
/// This file defines the basic set of abstractions to perform mathematical
/// operations between spatial quantities. For an introduction on spatial
/// quantities please refer to @ref multibody_spatial_algebra.
/// We follow some basic conventions and nomenclature used in the following
/// book by:
///   - Jain, A., 2010. Robot and multibody dynamics: analysis and algorithms.
///     Springer Science & Business Media.
///
/// Hereafter in this file we'll refer to this book by [Jain, 2010].

#include <limits>
#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {

/// This class is used to represent physical quantities that correspond to
/// a pair of three dimensional vectors. These include spatial velocities,
/// spatial accelerations and, spatial forces.
/// See section @ref multibody_spatial_vectors for a detailed description of the
/// concept of spatial vectors.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
template <typename T>
class SpatialVector {
 public:
  /// Sizes for spatial quantities and its components in three dimensions.
  enum {
    kSpatialVectorSize = 6,
    kRotationSize = 3,
    kTranslationSize = 3
  };
  /// The type of the underlying in-memory representation using an Eigen vector.
  typedef Vector6<T> CoeffsEigenType;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpatialVector)

  /// Default constructor. In Release builds the elements of the newly
  /// constructed spatial velocity are left uninitialized resulting in a zero
  /// cost operation. However in Debug builds those entries are set to NaN so
  /// that operations using this uninitialized spatial velocity fail fast,
  /// allowing fast bug detection.
  SpatialVector() {
    DRAKE_ASSERT_VOID(SetNaN());
  }

  /// SpatialVector constructor from an angular component @p w and a linear
  /// component @p v.
  SpatialVector(const Vector3<T>& w, const Vector3<T>& v) {
    V_.template head<3>() = w;
    V_.template tail<3>() = v;
  }

  /// The total size of the concatenation of the angular and linear components.
  /// In three dimensions this is six (6) and it is known at compile time.
  int size() const { return kSpatialVectorSize; }

  /// Const access to the underlying Eigen vector.
  const CoeffsEigenType& get_coeffs() const { return V_;}

  /// Const access to the i-th component of this spatial vector.
  /// Bounds are only checked in Debug builds for a zero overhead implementation
  /// in Release builds.
  const T& operator[](int i) const {
    DRAKE_ASSERT(0 <= i && i < kSpatialVectorSize);
    return V_[i];
  }

  /// Mutable access to the i-th component of this spatial vector.
  /// Bounds are only checked in Debug builds for a zero overhead implementation
  /// in Release builds.
  T& operator[](int i) {
    DRAKE_ASSERT(0 <= i && i < kSpatialVectorSize);
    return V_[i];
  }

  /// Const access to the angular component of this spatial vector.
  const Vector3<T>& angular() const {
    return *reinterpret_cast<const Vector3<T>*>(V_.data());
  }

  /// Mutable access to the angular component of this spatial vector.
  Vector3<T>& angular() {
    return *reinterpret_cast<Vector3<T>*>(V_.data());
  }

  /// Const access to the linear component of this spatial vector.
  const Vector3<T>& linear() const {
    return *reinterpret_cast<const Vector3<T>*>(
        V_.data() + kRotationSize);
  }

  /// Mutable access to the linear component of this spatial vector.
  Vector3<T>& linear() {
    return *reinterpret_cast<Vector3<T>*>(
        V_.data() + kRotationSize);
  }

  /// Performs the dot product in ℝ⁶ of `this` spatial vector with @p V.
  T dot(const SpatialVector& V) const {
    return V_.dot(V.V_);
  }

  /// Returns a (const) bare pointer to the underlying data.
  const T* data() const { return V_.data(); }

  /// Returns a (mutable) bare pointer to the underlying data.
  T* mutable_data() { return V_.data(); }

  /// @returns `true` if `other` is within a precision given by @p tolerance.
  /// The comparison is performed by comparing the angular (linear) component of
  /// `this` spatial vector with the angular (linear) component of @p other
  /// using the fuzzy comparison provided by Eigen's method isApprox().
  bool IsApprox(const SpatialVector<T>& other,
                double tolerance = Eigen::NumTraits<T>::epsilon()) {
    return linear().isApprox(other.linear(), tolerance) &&
           angular().isApprox(other.angular(), tolerance);
  }

  /// Sets all entries in `this` SpatialVector to NaN. Typically used to quickly
  /// detect uninitialized values since NaN will trigger a chain of invalid
  /// computations that can then be tracked back to the source.
  void SetNaN() {
    V_.setConstant(std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN());
  }

  /// Given `this` spatial velocity `V_AB_E` of a frame `B` measured in a frame
  /// `A` and expressed in a frame `E`, this method computes the spatial
  /// velocity of a frame `Q` rigidly moving with `B` but offset by a vector
  /// `r_BQ` from the orgin of frame `B` to the origin of frame Q.
  ///
  /// The operation performed, in vector free form, is: <pre>
  ///   w_AQ = w_AB,  i.e. the angular velocity of frame B and Q is the same.
  ///   v_AQ = v_AB + w_AB x r_BQ
  /// </pre>
  ///
  /// All quantities above must be expressed in a common frame `E` i.e: <pre>
  ///   w_AQ_E = w_AB_E
  ///   v_AQ_E = v_AB_E + w_AB_E x r_BQ_E
  /// </pre>
  ///
  /// This operation is performed in-place modifying the original object.
  ///
  /// @param[in] r_BQ_E Shift vector from `Bo` to `Qo` and expressed in
  ///                   frame `E`.
  /// @retval V_AQ_E The spatial velocity of frame `Q` with respect to `A` and
  ///                expressed in frame `A`.
  ///
  /// @see Shift() to compute the shifted spatial velocity without modifying
  ///      this original object.
  SpatialVector<T>& ShiftInPlace(const Vector3<T>& r_BQ_E) {
    /* The angular velocity remains the same. */
    /* For the linear velocity we have: v_AQ = v_AB + w_AB.cross(r_BQ). */
    linear() += angular().cross(r_BQ_E);
    return *this;
  }

  /// Given `this` spatial velocity `V_AB_E` of a frame `B` measured in a frame
  /// `A` and expressed in a frame `E`, this method computes the spatial
  /// velocity of a frame `Q` rigidly moving with `B` but offset by a vector
  /// `r_BQ` from the orgin of frame `B` to the origin of frame Q.
  ///
  /// The operation performed, in vector free form, is: <pre>
  ///   w_AQ = w_AB,  i.e. the angular velocity of frame B and Q is the same.
  ///   v_AQ = v_AB + w_AB x r_BQ
  /// </pre>
  ///
  /// All quantities above must be expressed in a common frame `E` i.e: <pre>
  ///   w_AQ_E = w_AB_E
  ///   v_AQ_E = v_AB_E + w_AB_E x r_BQ_E
  /// </pre>
  ///
  /// @param[in] r_BQ_E Shift vector from `Bo` to `Qo` and expressed in
  ///                   frame `E`.
  /// @retval V_AQ_E The spatial velocity of frame `Q` with respect to `A` and
  ///                expressed in frame `A`.
  ///
  /// @see ShiftInPlace() to compute the shifted spatial velocity in-place
  ///      modifying the original object.
  SpatialVector<T> Shift(const Vector3<T>& r_BQ_E) const {
    return SpatialVector<T>(*this).ShiftInPlace(r_BQ_E);
  }

 private:
  CoeffsEigenType V_;
};

/// Insertion operator to write SpatialVectors into a `std::ostream`.
/// Especially useful for debugging.
template <typename T> inline
std::ostream& operator<<(std::ostream& o, const SpatialVector<T>& V) {
  o << "[" << V[0];
  for (int i = 1; i < V.size(); ++i) o << ", " << V[i];
  o << "]^T";  // The "transpose" symbol.
  return o;
}

/// Multiplication of a SpatialVector from the left by a scalar.
template <typename T>
inline SpatialVector<T> operator*(
    const T& s, const SpatialVector<T>& V) {
  return SpatialVector<T>(s * V.get_coeffs());
}

}  // namespace multibody
}  // namespace drake
