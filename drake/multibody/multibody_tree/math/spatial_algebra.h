#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {

/// This class is used to represent physical quantities that can be expressed as
/// a pair of three dimensional vectors. These include spatial velocities,
/// spatial accelerations and spatial forces.
/// See section @ref multibody_spatial_vectors for a detailed description of the
/// concept of spatial vectors.
///
/// @tparam T The unerlying scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template <typename T>
class SpatialVector {
 public:
  // Sizes for spatial quantities and its components in three dimensions.
  enum {
    kSpatialVectorSize = 6,
    kSpatialVectorAngularSize = 3,
    kSpatialVectorLinearSize = 3
  };
  // The type of the underlying in-memory representation using an Eigen vector.
  typedef Eigen::Matrix<T, kSpatialVectorSize, 1> CoeffsEigenType;

  /// Default constructor leaving numerical entries un-initialized to avoid any
  /// computational cost.
  SpatialVector() {}

  /// SpatialVector constructor from an angular component @p w and a linear
  /// component @p v.
  SpatialVector(const Vector3<T>& w, const Vector3<T>& v) {
    V_ << w, v;
  }

  /// The total size of the concatenation of the angular and linear components.
  /// In three dimension this is six (6) and it is known at compile time.
  int size() const { return kSpatialVectorSize; }

  /// The size of the angular component.
  /// In three dimension this is three (3) and it is known at compile time.
  int angular_size() const { return kSpatialVectorAngularSize; }

  /// The size of the linear component.
  /// In three dimension this is three (3) and it is known at compile time.
  int linear_size() const { return kSpatialVectorLinearSize; }

  /// Const access to the underlying Eigen vector.
  const CoeffsEigenType& get_coeffs() const { return V_;}

  /// Const access to the i-th component of this spatial vector.
  /// Bounds are only checked in Debug builds for a zero overhead implementation
  /// in Release builds.
  const T& operator[](int i) const
  {
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
        V_.data() + kSpatialVectorAngularSize);
  }

  /// Mutable access to the linear component of this spatial vector.
  Vector3<T>& linear() {
    return *reinterpret_cast<Vector3<T>*>(
        V_.data() + kSpatialVectorAngularSize);
  }

  /// Performs the dot product in R^6 of `this` spatial vector with @p other.
  T dot(const SpatialVector& V) const {
    return V_.dot(V.V_);
  }

  /// Returns a (const) bare pointer to the unerlying data.
  const T* data() const { return V_.data(); }

  /// Returns a (mutable) bare pointer to the unerlying data.
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

 private:
  CoeffsEigenType V_;

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

/// Insertion operator to write SpatialVectors into a `std::ostream`.
/// Especially useful for debugging.
template <typename T> inline
std::ostream& operator<<(std::ostream& o, const SpatialVector<T>& V) {
  o << "[" << V[0];
  for(int i = 1; i < V.size(); ++i) o << ", " << V[i];
  o << "]^T";  // The "transpose" symbol.
  return o;
}

/// Multiplication of a SpatialVector from the left by a scalar.
template <typename T>
inline SpatialVector<T> operator*(
    const T& s, const SpatialVector<T>& V)
{
  return SpatialVector<T>(s * V.angular(), s * V.linear());
}

// Forward declaration of the ShiftOperator's transpose.
template <typename T> class ShiftOperatorTranspose;

/// A class representing the Rigid Body Transformation Matrix as defined in
/// Section 1.4 of A Jain's book.
/// Given a pair of frames `x` and `y`
///
/// @tparam T
template <typename T>
class ShiftOperator {
 public:
  /// Default constructor leaves the offset uninitialized resulting in a zero
  /// cost operation.
  ShiftOperator() {}

  /// Constructs a rigid body transformation operator between a pair of frames
  /// `X` and `Y` given a vector @p offset_XoYo_F from `Xo` to `Yo`.
  /// For a vector @p offset_XoYo_F expressed in a frame `F`, this operator can
  /// only operate on spatial vectors expressed in the same frame `F`.
  /// @param[in] offset_XoYo_F Vector from `Xo` to `Yo` expressed in an implicit
  ///                          frame `F`.
  ShiftOperator(const Vector3<T>& offset_XoYo_F) : offset_(offset_XoYo_F) {}

  // Returns the vector from the origin of frame X to the origin of frame Y
  // expressed in the implicit frame F.
  const Vector3<T>& offset() const { return offset_; }

  ShiftOperatorTranspose<T> transpose() const;

  /// Creates a %ShiftOperator initialized with a NaN offset vector.
  static ShiftOperator<T> NaN() {
    return ShiftOperator<T>(Vector3<T>::Constant(std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN()));
  }

  /// Sets the offset of this operator to NaN. Typically used to quickly detect
  /// uninitialized values since NaN will trigger a chain of invalid
  /// computations that then can be tracked to the source.
  void SetToNaN() {
    offset_.setConstant(std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN());
  }

 private:
  Vector3<T> offset_;

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

template <typename T>
class ShiftOperatorTranspose {
public:
  explicit ShiftOperatorTranspose(const ShiftOperator<T>& phi_XY_F) :
      phi_(phi_XY_F) {}

  // Returns the vector from the origin of frame X to the origin of frame Y
  // expressed in the implicit frame F.
  const Vector3<T>& offset() const { return phi_.offset(); }

  /// Given the spatial velocity `V_AB` of a frame `B` with respect to a frame 
  /// `A`, compute the spatial velocity of a frame `Q` rigidly moving with `B` 
  /// but offset by vector `r_BQ`.
  ///
  /// The operation performed, in vector free form, is:
  ///   w_AQ = w_AB,  i.e. the angular velocity of frame B and Q is the same.
  ///   v_AQ = v_AB + w_AB x r_BQ
  ///
  /// All quantities above must be expressed in the same frame `E` i.e:
  ///   w_AQ_E = w_AB_E
  ///   v_AQ_E = v_AB_E + w_AB_E x r_BQ_E
  ///
  /// @param[in] V_AB_E Spatial velocity of frame `B` with respect to frame `A`,
  /// expressed in frame `E`.
  /// @param[in] r_BQ_E Shift vector from `Bo` to `Qo`, expressed in frame `E`.
  /// @returns V_AQ_E The spatial velocity of frame `Q` with respect to `A` and
  /// expressed in frame `A`.
  static SpatialVector<T> ShiftSpatialVelocity(
      const SpatialVector<T>& V_AB_E, const Vector3<T>& r_BQ_E) {
    return SpatialVector<T>(
        /* Same angular velocity. */
        V_AB_E.angular(),
        /* Linear velocity v_AQ = v_AB + w_AB.cross(r_BQ). */
        V_AB_E.linear() + V_AB_E.angular().cross(r_BQ_E));
  }

 private:
  const ShiftOperator<T>& phi_;
};

template <typename T>
inline ShiftOperatorTranspose<T> ShiftOperator<T>::transpose() const {
  return ShiftOperatorTranspose<T>(*this);
}

/// Given the spatial velocity `V_AB` of a frame `B` measured in a frame `A`, 
/// compute the spatial velocity of a frame `Q` rigidly moving with `B` 
/// but offset by vector `r_BQ`.
///
/// The operation performed, in vector free form, is:
///   V_AQ = phi_BQ^T * V_AB
///
/// All quantities above must be expressed in the same frame `E` i.e:
///   V_AQ_E = phi_BQ_E^T * V_AB_E
///
/// @param[in] phiT_BQ_E Transpose of the shift operator from frame `B` to `Q`
/// expressed in a frame `E`.
/// @param[in] V_AB_E The spatial velocity of frame `B` measured in `A` 
/// and expressed in `E`.
/// @returns V_AQ_E The spatial velocity of frame `Q` measured in `A` and 
/// expressed in `E`.
template <typename T>
inline SpatialVector<T> operator*(
    const ShiftOperatorTranspose<T>& phiT_BQ_E, const SpatialVector<T>& V_AB_E)
{
  return ShiftOperatorTranspose<T>::ShiftSpatialVelocity(
      V_AB_E, phiT_BQ_E.offset());
}

}  // namespace multibody
}  // namespace drake
