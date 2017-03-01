#pragma once

#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {

namespace internal { 
// Traits class to figure out compile-time quantities used within SpatialVector,
// specifically the type T of the Eigen compatible scalar type the spatial
// quantity is instantiated with.
// Specific spatial quantities derived from SpatialVector need to define these 
// traits within this internal namespace.
// Users do not need to interact with these traits.
template <class SpatialQuantity> struct spatial_vector_traits;
};

/// This class is used to represent physical quantities that correspond to
/// spatial vectors such as spatial velocities, spatial accelerations and 
/// spatial forces. Spatial vectors are 6-element quantities that are
/// pairs of ordinary 3-vectors. Elements 0-2 are always the rotational 
/// component while elements 3-5 are always the translational component.
/// For a more detailed introduction on spatial vectors please refer to
/// section @ref multibody_spatial_vectors.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
template <typename Derived>
class SpatialVector {
  typedef typename internal::spatial_vector_traits<Derived>::ScalarType T;
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpatialVector)

  /// Sizes for spatial quantities and its components in three dimensions.
  enum {
    kSpatialVectorSize = 6,
    kRotationSize = 3,
    kTranslationSize = 3
  };
  /// The type of the underlying in-memory representation using an Eigen vector.
  typedef Vector6<T> CoeffsEigenType;

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
  /// This constructor will assert the size of `V` is six (6) at compile-time
  /// for fixed sized Eigen expressions and at run-time for dynamic sized Eigen
  /// expressions.
  template <typename OtherDerived>
  explicit SpatialVector(const Eigen::MatrixBase<OtherDerived>& V) : V_(V) {}

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

  /// @returns `true` if `other` is within a precision given by @p tolerance.
  /// The comparison is performed by comparing the angular (linear) component of
  /// `this` spatial vector with the angular (linear) component of @p other
  /// using the fuzzy comparison provided by Eigen's method isApprox().
  bool IsApprox(const Derived& other,
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

 protected:
  CoeffsEigenType& get_coeffs() { return V_;}

  const CoeffsEigenType& get_coeffs() const { return V_;}

 private:
  CoeffsEigenType V_;
};

/// Stream insertion operator to write SpatialVector objects into a
/// `std::ostream`. Especially useful for debugging.
/// @relates SpatialVector.
template <typename Derived> inline
std::ostream& operator<<(std::ostream& o, const SpatialVector<Derived>& V) {
  o << "[" << V[0];
  for (int i = 1; i < V.size(); ++i) o << ", " << V[i];
  o << "]ᵀ";  // The "transpose" symbol.
  return o;
}

}  // namespace multibody
}  // namespace drake
