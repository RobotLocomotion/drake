#pragma once

/// @file
/// This file defines the basic set of abstractions to perform mathematical
/// operations between spatial quantities. For an introduction on spatial
/// quantities please refer to @ref multibody_spatial_algebra.
/// We follow some basic conventions and nomenclature used in his book by
/// Abhinandan Jain:
///   Jain, A., 2010. Robot and multibody dynamics: analysis and algorithms.
///   Springer Science & Business Media.
/// Hereafter in this file we'll refer to this book by Jain (2010).

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
template <typename T>
class SpatialVector {
 public:
  /// Sizes for spatial quantities and its components in three dimensions.
  enum {
    kSpatialVectorSize = 6,
    kSpatialVectorAngularSize = 3,
    kSpatialVectorLinearSize = 3
  };
  /// The type of the underlying in-memory representation using an Eigen vector.
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
/// Section 1.4 of Jain (2010)'s book.
/// Given a frame @f$ A @f$ that moves with spatial velocity @f$ ^WV^A @f$
/// measured in a frame @f$ W @f$ and a second frame @f$ B @f$ that **rigidly**
/// moves with frame @f$ A @f$, the shift operator @f$ \phi(A,B) @f$ is defined
/// such that the spatial velocity of
/// frame @f$ B @f$ as measured in this same frame @f$ W @f$ is given by:
/// @f[
///   ^WV^B = \phi(A,B)^T \, ^WV^A
/// @f]
/// where the superscript @f$ ^T @f$ represents the _transpose_ operation of the
/// shift operator.
///
/// The above example reads in code as:
/// @code
/// Assume the following quantities available within a given scope.
/// // Spatial velocity of frame A measured in frame W, expressed in a third
/// // frame E.
/// SpatialVector<T> V_WA_E;
/// // Vector from A's origin to B's origin, expressed in E.
/// Vector3<T>       p_AB_E;
/// // The spatial velocity of frame B rigidly moving with frame A is obtained
/// // using the shift operator (its transpose) from A to B.
/// SpatialVector<T> V_WB_E = ShiftOperator(p_AB_E).transpose() * V_WA_E;
/// @endcode
///
/// Notice we are using the monogram notation introduced in
/// @ref multibody_notation_basics which allows us to write unambiguous
/// expressions in text code. In addition, when needed, we are explicit about
/// the frame these quantities are expressed in.
///
/// @note This class does not implement any mechanism to check the consistency
/// between the frames in which the shift operator itself and the spatial
/// vectors it applies to are described.
///
/// With spatial forces defined in @ref multibody_spatial_vectors as the pair of
/// of three dimensional vectors @f$ \tau @f$ for the torque component and
/// @f$ f @f$ for the force component, the shift operator relates the
/// spatial force about a frame @f$ A @f$ on a rigid body with the spatial
/// force about a frame @f$ B @f$ on the same rigid body as:
/// @f[
///   ^WF^A = \phi(A,B)\, ^WF^B
/// @f]
/// which implies that the spatial force @f$ ^WF^B @f$ about @f$ B @f$ on a
/// rigid body is equivalent to the spatial force
/// @f$ ^WF^A = \phi(A,B)\, ^WF^B @f$ about @f$ A @f$ on the same rigid body.
///
/// Some basic group properties of this operator are:
/// @f{eqnarray*}{
///   \phi(X,X) &=& \mathcal{I},\quad
///                 \text{with } \mathcal{I}\text{ the identity operator.}\\
///   \phi(X,Z) &=& \phi(X,Y) \, \phi(Y,Z)\\
///   \phi(X,Y)^{-1} &=& \phi(Y,X)
/// @f}
///
/// @see ShiftOperatorTranspose for the dual version of this operator which
///      allows transformation of spatial forces.
///
/// @tparam T The unerlying scalar type. Must be a valid Eigen scalar.
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

  /// Returns the vector from the origin of frame `X` to the origin of frame `Y`
  /// expressed in the implicit frame `F`.
  const Vector3<T>& offset() const { return offset_; }

  /// Returns the transpose of this operator which allows to transform spatial
  /// velocities between two frames rigidly attached to each other.
  ShiftOperatorTranspose<T> transpose() const;

  /// Creates a %ShiftOperator initialized with a NaN offset vector.
  static ShiftOperator<T> NaN() {
    return ShiftOperator<T>(Vector3<T>::Constant(std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN()));
  }

  /// Sets the offset of this operator to NaN. Typically used to quickly detect
  /// uninitialized values since NaN will trigger a chain of invalid
  /// computations that can then be tracked back to the source.
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

/// This class is the return type for ShiftOperator::transpose().
/// For a detailed description of the shift operator please refer to the
/// documentation for the ShiftOperator class.
///
/// @tparam T The unerlying scalar type. Must be a valid Eigen scalar.
template <typename T>
class ShiftOperatorTranspose {
public:
  /// Constructs the transpose of a given ShiftOperator `phi_XY_F` between two
  /// frames `X` and `Y`, expressed in a third frame `F`.
  explicit ShiftOperatorTranspose(const ShiftOperator<T>& phi_XY_F) :
      phi_(phi_XY_F) {}

  /// Returns the vector from the origin of frame `X` to the origin of frame `Y`
  /// expressed in the implicit frame `F`.
  const Vector3<T>& offset() const { return phi_.offset(); }

  /// Given the spatial velocity `V_AB` of a frame `B` measured in a frame
  /// `A`, this method computes the spatial velocity of a frame `Q` rigidly
  /// moving with `B` but offset by vector `r_BQ`.
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
  /// @param[in] V_AB_E Spatial velocity of frame `B` measured in frame `A`,
  ///                   expressed in frame `E`.
  /// @param[in] r_BQ_E Shift vector from `Bo` to `Qo` and expressed in
  ///                   frame `E`.
  /// @returns   V_AQ_E The spatial velocity of frame `Q` with respect to `A`
  ///                   and expressed in frame `A`.
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

// Implementation for ShiftOperator<T>::transpose().
template <typename T>
inline ShiftOperatorTranspose<T> ShiftOperator<T>::transpose() const {
  return ShiftOperatorTranspose<T>(*this);
}

/// Given the spatial velocity `V_AB` of a frame `B` measured in a frame `A`, 
/// compute the spatial velocity of a frame `Q` rigidly moving with `B` 
/// but offset by vector `r_BQ`.
///
/// The operation performed, in vector free form, is: <pre>
///   V_AQ = phiT_BQ * V_AB
/// </pre>
/// where `phiT_BQ` is the transpose of the ShiftOperator `phi_BQ`.
///
/// All quantities above must be expressed in a common frame `E` i.e: <pre>
///   V_AQ_E = phi_BQ_E^T * V_AB_E
/// </pre>
///
/// @param[in] phiT_BQ_E Transpose of the shift operator from frame `B` to `Q`
///                      expressed in a frame `E`.
/// @param[in] V_AB_E    The spatial velocity of frame `B` measured in `A`
///                      and expressed in `E`.
/// @returns V_AQ_E      The spatial velocity of frame `Q` measured in `A` and
///                      expressed in `E`.
template <typename T>
inline SpatialVector<T> operator*(
    const ShiftOperatorTranspose<T>& phiT_BQ_E, const SpatialVector<T>& V_AB_E)
{
  return ShiftOperatorTranspose<T>::ShiftSpatialVelocity(
      V_AB_E, phiT_BQ_E.offset());
}

}  // namespace multibody
}  // namespace drake
