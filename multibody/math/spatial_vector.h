#pragma once

#include <limits>
#include <tuple>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rotation_matrix.h"

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
/// @tparam_default_scalar
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

  /// Returns the maximum absolute values of the differences in the rotational
  /// and translational components of `this` and `other` (i.e., the infinity
  /// norms of the difference in rotational and translational components).
  /// These quantities are returned in a tuple, in the order below.
  /// std::tuple       | Description
  /// -----------------|-------------------------------------------------
  /// w_max_difference | Maximum absolute difference in rotation components
  /// v_max_difference | Maximum absolute difference in translation components
  std::tuple<const T, const T> GetMaximumAbsoluteDifferences(
      const SpatialQuantity& other) const {
    const Vector3<T> w_difference = rotational() - other.rotational();
    const Vector3<T> v_difference = translational() - other.translational();
    const T w_max_difference = w_difference.template lpNorm<Eigen::Infinity>();
    const T v_max_difference = v_difference.template lpNorm<Eigen::Infinity>();
    return std::make_tuple(w_max_difference, v_max_difference);
  }

  /// Compares the rotational and translational parts of `this` and `other`
  /// to check if they are the same to within specified absolute differences.
  /// @param[in] rotational_tolerance maximum allowable absolute difference
  /// between the rotational parts of `this` and `other`.  The units depend on
  /// the underlying class.  For example, spatial velocity, acceleration, and
  /// force have units of rad/sec, rad/sec^2, and N*m, respectively.
  /// @param[in] translational_tolerance maximum allowable absolute difference
  /// between the translational parts of `this` and `other`.  The units depend
  /// on the underlying class.  For example, spatial velocity, acceleration, and
  /// force have units of meter/sec, meter/sec^2, and Newton, respectively.
  /// @returns `true` if the rotational part of `this` and `other` are equal
  /// within @p rotational_tolerance and the translational part of `this` and
  /// `other` are equal within @p translational_tolerance.
  decltype(T() < T()) IsNearlyEqualWithinAbsoluteTolerance(
      const SpatialQuantity& other, double rotational_tolerance,
      double translational_tolerance) const {
    T w_max_difference, v_max_difference;
    std::tie(w_max_difference, v_max_difference) =
        GetMaximumAbsoluteDifferences(other);
    return w_max_difference <= rotational_tolerance &&
           v_max_difference <= translational_tolerance;
  }

  /// Compares `this` spatial vector to the provided spatial vector `other`
  /// within a specified tolerance.
  /// Mathematically, if `this` is the spatial vector U and `other` is the
  /// spatial vector V, then this method returns `true` if `‖U-V‖∞ < ε` and
  /// `false` otherwise.
  decltype(T() < T()) IsApprox(
      const SpatialQuantity& other,
      double tolerance = std::numeric_limits<double>::epsilon()) const {
    return IsNearlyEqualWithinAbsoluteTolerance(other, tolerance, tolerance);
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
  SpatialQuantity& SetZero() {
    V_.setZero();
    return get_mutable_derived();
  }

  /// Returns a reference to the underlying storage.
  CoeffsEigenType& get_coeffs() { return V_;}

  /// Returns a constant reference to the underlying storage.
  const CoeffsEigenType& get_coeffs() const { return V_;}

  /// Unary minus operator.
  SpatialQuantity operator-() const {
    return SpatialQuantity(-get_coeffs());
  }

  /// Addition assignment operator.
  SpatialQuantity& operator+=(const SpatialQuantity& V) {
    this->get_coeffs() += V.get_coeffs();
    return get_mutable_derived();
  }

  /// Subtraction assignment operator.
  SpatialQuantity& operator-=(const SpatialQuantity& V) {
    this->get_coeffs() -= V.get_coeffs();
    return get_mutable_derived();
  }

  /// Multiplication assignment operator.
  SpatialQuantity& operator*=(const T& s) {
    this->get_coeffs() *= s;
    return get_mutable_derived();
  }

  /// (Advanced) Addition operator. Implements the addition of V1 and V2 as
  /// elements in ℝ⁶.
  /// @warning This operation might not be physical for certain spatial
  /// quantities. For instace, combining the spatial accelerations of two frames
  /// does not correspond to this operation.
  friend SpatialQuantity operator+(const SpatialQuantity& V1,
                                   const SpatialQuantity& V2) {
    return SpatialQuantity(V1) += V2;
  }

  /// (Advanced) Subtraction operator. Implements the subtraction of V1 and V2
  /// as elements in ℝ⁶.
  /// @warning This operation might not be physical for certain spatial
  /// quantities.
  friend SpatialQuantity operator-(const SpatialQuantity& V1,
                                   const SpatialQuantity& V2) {
    return SpatialQuantity(V1) -= V2;
  }

  /// Multiplication of a spatial vector V from the left by a scalar `s`.
  /// @relates SpatialVector.
  friend SpatialQuantity operator*(const T& s, const SpatialQuantity& V) {
    return SpatialQuantity(V) *= s;
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
      const math::RotationMatrix<T>& R_FE, const SpatialQuantity& V_E) {
    return SpatialQuantity(R_FE * V_E.rotational(), R_FE * V_E.translational());
  }

  /// Factory to create a _zero_ %SpatialVector, i.e. rotational and
  /// translational components are both zero.
  static SpatialQuantity Zero() {
    return SpatialQuantity{}.SetZero();
  }

 private:
  // Helper method to return a mutable reference to the derived spatial
  // quantity.
  SpatialQuantity& get_mutable_derived() {
    // Static cast is safe since types are resolved at compile time by CRTP.
    return *static_cast<SpatialQuantity*>(this);
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
