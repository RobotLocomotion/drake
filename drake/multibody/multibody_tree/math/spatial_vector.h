#pragma once

#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {

/// This class is used to represent physical quantities that correspond to
/// spatial vectors such as spatial velocities, spatial accelerations and
/// spatial forces. Spatial vectors are 6-element quantities that are
/// pairs of ordinary 3-vectors. Elements 0-2 are always the rotational
/// component while elements 3-5 are always the translational component.
/// For a more detailed introduction on spatial vectors please refer to
/// section @ref multibody_spatial_vectors.
///
/// @tparam SV The type of the more specialized spatial vector class.
///            It must be a template on the scalar type T.
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
template <template <typename> class SV, typename T>
class SpatialVector {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpatialVector)

  /// The more specialized spatial vector class templated on the scalar
  /// type T.
  using SpatialQuantity = SV<T>;

  /// Sizes for spatial quantities and its components in three dimensions.
  enum {
    kSpatialVectorSize = 6,
    kRotationSize = 3,
    kTranslationSize = 3
  };

  /// The type of the underlying in-memory representation using an Eigen vector.
  typedef Vector6<T> CoeffsEigenType;

  // Make it available to implementations using SpatialVector.
  typedef T ScalarType;

  /// Default constructor. In Release builds the elements of the newly
  /// constructed spatial vector are left uninitialized resulting in a zero
  /// cost operation. However in Debug builds those entries are set to NaN so
  /// that operations using this uninitialized spatial vector fail fast,
  /// allowing fast bug detection.
  SpatialVector() {
    DRAKE_ASSERT_VOID(SetNaN());
  }

  /// SpatialVector constructor from an rotational component @p w and a linear
  /// component @p v.
  SpatialVector(const Eigen::Ref<const Vector3<T>>& w,
                const Eigen::Ref<const Vector3<T>>& v) {
    V_.template head<3>() = w;
    V_.template tail<3>() = v;
  }

  /// SpatialVector constructor from an Eigen expression that represents a
  /// six-dimensional vector.
  /// This constructor will assert the size of V is six (6) at compile-time
  /// for fixed sized Eigen expressions and at run-time for dynamic sized Eigen
  /// expressions.
  template <typename OtherDerived>
  explicit SpatialVector(const Eigen::MatrixBase<OtherDerived>& V) : V_(V) {}

  /// Creates a copy of `this` spatial vector with its rotational component set
  /// to `rotational`.
  SpatialQuantity with_rotational(
      const Eigen::Ref<const Vector3<T>>& rotational) const {
    SpatialQuantity V_with_rotational(get_derived());
    V_with_rotational.rotational() = rotational;
    return V_with_rotational;
  }

  /// Creates a copy of `this` spatial vector with its translational component
  /// set to `translational`.
  SpatialQuantity with_translational(
      const Eigen::Ref<const Vector3<T>>& translational) const {
    SpatialQuantity V_with_translational(get_derived());
    V_with_translational.translational() = translational;
    return V_with_translational;
  }

  /// The total size of the concatenation of the angular and linear components.
  /// In three dimensions this is six (6) and it is known at compile time.
  int size() const { return kSpatialVectorSize; }

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

  /// Const access to the rotational component of this spatial vector.
  const Vector3<T>& rotational() const {
    // We are counting on a particular representation for an Eigen Vector3<T>:
    // it must be represented exactly as 3 T's in an array with no metadata.
    return *reinterpret_cast<const Vector3<T>*>(V_.data());
  }

  /// Mutable access to the rotational component of this spatial vector.
  Vector3<T>& rotational() {
    // We are counting on a particular representation for an Eigen Vector3<T>:
    // it must be represented exactly as 3 T's in an array with no metadata.
    return *reinterpret_cast<Vector3<T>*>(V_.data());
  }

  /// Const access to the translational component of this spatial vector.
  const Vector3<T>& translational() const {
    // We are counting on a particular representation for an Eigen Vector3<T>:
    // it must be represented exactly as 3 T's in an array with no metadata.
    return *reinterpret_cast<const Vector3<T>*>(
        V_.data() + kRotationSize);
  }

  /// Mutable access to the translational component of this spatial vector.
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

  /// Compares `this` spatial vector to the provided spatial vector `other`
  /// within a specified precision.
  /// @returns `true` if `other` is within a precision given by @p tolerance.
  /// The comparison is performed by comparing the translational component of
  /// `this` spatial vector with the rotational component of @p other
  /// using the fuzzy comparison provided by Eigen's method isApprox().
  bool IsApprox(const SpatialQuantity& other,
                double tolerance = Eigen::NumTraits<T>::epsilon()) const {
    return translational().isApprox(other.translational(), tolerance) &&
           rotational().isApprox(other.rotational(), tolerance);
  }

  /// Sets all entries in `this` SpatialVector to NaN. Typically used to
  /// quickly detect uninitialized values since NaN will trigger a chain of
  /// invalid computations that can then be tracked back to the source.
  void SetNaN() {
    V_.setConstant(std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN());
  }

  /// Sets both rotational and translational components of `this`
  /// %SpatialVector to zero.
  void SetZero() {
    V_.setZero();
  }

  /// Returns a reference to the underlying storage.
  CoeffsEigenType& get_coeffs() { return V_;}

  /// Returns a constant reference to the underlying storage.
  const CoeffsEigenType& get_coeffs() const { return V_;}

  /// Multiplication of a spatial vector V from the left by a scalar `s`.
  /// @relates SpatialVector.
  friend SpatialQuantity operator*(const T& s, const SpatialQuantity& V) {
    return SpatialQuantity(s * V.get_coeffs());
  }

  /// Multiplication of a spatial vector V from the right by a scalar `s`.
  /// @relates SpatialVector.
  friend SpatialQuantity operator*(const SpatialQuantity& V, const T& s) {
    return s * V;  // Multiplication by scalar is commutative.
  }

  /// This operation re-expresses the spatial vector `V_E` originally expressed
  /// in frame E, into `V_F`, the same spatial vector expresed in another frame
  /// F. The transformation requires the rotation matrix `R_FE` representing the
  /// orientation of the original frame E with respect to frame F.
  /// The operation performed is: <pre>
  ///   V_F.rotational()    = R_FE * V_E.rotational(),
  ///   V_F.translational() = R_FE * V_E.translational()
  /// </pre>
  /// @returns V_F The same spatial vector re-expressed in frame F.
  friend SpatialQuantity operator*(
      const Matrix3<T>& R_FE, const SpatialQuantity& V_E) {
    return SpatialQuantity(R_FE * V_E.rotational(), R_FE * V_E.translational());
  }

 private:
  // Helper method to return a const reference to the derived spatial quantity.
  const SpatialQuantity& get_derived() const {
    // Static cast is safe since types are resolved at compile time by CRTP.
    return *static_cast<const SpatialQuantity*>(this);
  }

  CoeffsEigenType V_;
};

/// Stream insertion operator to write SpatialVector objects into a
/// `std::ostream`. Especially useful for debugging.
/// @relates SpatialVector.
template <template <typename> class SpatialQuantity, typename T> inline
std::ostream& operator<<(std::ostream& o,
                         const SpatialVector<SpatialQuantity, T>& V) {
  o << "[" << V[0];
  for (int i = 1; i < V.size(); ++i) o << ", " << V[i];
  o << "]ᵀ";  // The "transpose" symbol.
  return o;
}

}  // namespace multibody
}  // namespace drake
