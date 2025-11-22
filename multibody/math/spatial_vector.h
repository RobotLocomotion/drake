#pragma once

#ifndef DRAKE_SPATIAL_ALGEBRA_HEADER
#error Please include "drake/multibody/math/spatial_algebra.h", not this file.
#endif

#include <limits>
#include <tuple>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/fmt_ostream.h"
#include "drake/math/rotation_matrix.h"

/* Note: many of the operations in the various SpatialVector-derived classes
are annotated with operation counts that look like "// 33 flops". These are
counts of the _logical_ number of floating point operations required for
the computation, e.g. 9 for a cross product, 45 for the product of two
3x3 matrices. That is not necessarily the same as the number of instructions
executed since SIMD instructions can perform multiple operations. These are
intended as a hint as to where expensive work is being done when looking to
speed up computation. Be sure to do real A/B performance measurements to
determine whether a hypothetical improvement is really faster. */

namespace drake {
namespace multibody {

/// This class represents a _spatial vector_ and has 6 elements, with a
/// 3-element rotational vector on top of a 3-element translational vector.
/// Important subclasses of %SpatialVector include SpatialVelocity,
/// SpatialAcceleration, SpatialForce, and SpatialMomentum.
/// Each of the 3-element vectors is assumed to be expressed in the same
/// _expressed-in_ frame E. This class only stores 6 elements and does not store
/// the underlying expressed-in frame E or other information. The user is
/// responsible for explicitly tracking the underlying frames with
/// @ref multibody_quantities "monogram notation". For example, Foo_E denotes
/// an arbitrary spatial vector Foo expressed in a frame E. Details on spatial
/// vectors and monogram notation are in sections @ref multibody_spatial_vectors
/// and @ref multibody_quantities "monogram notation".
///
/// @tparam SV The type of the more specialized spatial vector class.
///            It must be a template on the scalar type T.
/// @tparam_default_scalar
template <template <typename> class SV, typename T>
class SpatialVector {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpatialVector);

  /// The more specialized spatial vector class templated on the scalar type T.
  using SpatialQuantity = SV<T>;

  /// Sizes for spatial quantities and its components in 3D (three dimensions).
  enum {
    kSpatialVectorSize = 6,  // BR
    kRotationSize = 3,       //
    kTranslationSize = 3
  };

  /// The type of the underlying in-memory representation using an Eigen vector.
  using CoeffsEigenType = Vector6<T>;

  /// Default constructor. In Release builds, all 6 elements of a newly
  /// constructed spatial vector are uninitialized (for speed).  In Debug
  /// builds, the 6 elements are set to NaN so that invalid operations on an
  /// uninitialized spatial vector fail fast (fast bug detection).
  SpatialVector() { DRAKE_ASSERT_VOID(SetNaN()); }

  /// Constructs a spatial vector from a rotational component w and a
  /// translational component v.
  SpatialVector(const Eigen::Ref<const Vector3<T>>& w,
                const Eigen::Ref<const Vector3<T>>& v) {
    V_.template head<3>() = w;
    V_.template tail<3>() = v;
  }

  /// Constructs a spatial vector V from an Eigen expression that represents a
  /// 6-element vector (3-element rotational vector on top of a 3-element
  /// translational vector). This constructor asserts the size of V is six (6)
  /// either at compile-time for fixed sized Eigen expressions or at run-time
  /// for dynamic sized Eigen expressions.
  template <typename OtherDerived>
  explicit SpatialVector(const Eigen::MatrixBase<OtherDerived>& V) : V_(V) {}

  /// For 3D (three-dimensional) analysis, the total size of the concatenated
  /// rotational vector (3 elements) and translational vector (3 elements) is
  /// six (6), which is known at compile time.
  int size() const { return kSpatialVectorSize; }

  /// Const access to the i-th element of this spatial vector. In Debug
  /// builds, this function asserts that i is in bounds whereas for
  /// release builds, no bounds-check on i is performed (for speed).
  const T& operator[](int i) const {
    DRAKE_ASSERT(0 <= i && i < kSpatialVectorSize);
    return V_[i];
  }

  /// Mutable access to the i-th element of this spatial vector. In Debug
  /// builds, this function asserts that i is in bounds whereas for
  /// release builds, no bounds-check on i is performed (for speed).
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
    return *reinterpret_cast<const Vector3<T>*>(V_.data() + kRotationSize);
  }

  /// Mutable access to the translational component of this spatial vector.
  Vector3<T>& translational() {
    // We are counting on a particular representation for an Eigen Vector3<T>:
    // it must be represented exactly as 3 T's in an array with no metadata.
    return *reinterpret_cast<Vector3<T>*>(V_.data() + kRotationSize);
  }

  /// Returns a (const) bare pointer to the underlying data. It is guaranteed
  /// that there will be six (6) T's densely packed at data[0], data[1], etc.
  const T* data() const { return V_.data(); }

  /// Returns a (mutable) bare pointer to the underlying data. It is guaranteed
  /// that there will be six (6) T's densely packed at data[0], data[1], etc.
  T* mutable_data() { return V_.data(); }

  /// Returns the maximum absolute values of the differences in the rotational
  /// and translational components of `this` and `other` (i.e., the infinity
  /// norms of the difference in rotational and translational components).
  /// @param[in] other spatial vector to subtract from `this` spatial vector.
  /// @returns The following quantities in a tuple, in the order below.
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
  /// @param[in] other spatial vector to compare to `this` spatial vector.
  /// @param[in] rotational_tolerance maximum allowable absolute difference
  /// between the rotational parts of `this` and `other`.  The units depend on
  /// the underlying class.  For example, spatial velocity, acceleration, and
  /// force have units of rad/sec, rad/sec^2, and N*m, respectively.
  /// @param[in] translational_tolerance maximum allowable absolute difference
  /// between the translational parts of `this` and `other`.  The units depend
  /// on the underlying class.  For example, spatial velocity, acceleration, and
  /// force have units of meter/sec, meter/sec^2, and Newton, respectively.
  /// @returns true if all three rotational elements of `this` and `other` are
  /// equal within rotational_tolerance and all three translational elements of
  /// `this` and `other` are equal within translational_tolerance.
  decltype(T() < T()) IsNearlyEqualWithinAbsoluteTolerance(
      const SpatialQuantity& other, double rotational_tolerance,
      double translational_tolerance) const {
    T w_max_difference, v_max_difference;
    std::tie(w_max_difference, v_max_difference) =
        GetMaximumAbsoluteDifferences(other);
    return w_max_difference <= rotational_tolerance &&
           v_max_difference <= translational_tolerance;
  }

  /// Determines whether all six corresponding elements of two spatial vectors
  /// are equal to each other to within a specified tolerance epsilon.
  /// @param[in] other spatial vector to compare to `this` spatial vector.
  /// @param[in] tolerance specified tolerance for this test.
  /// @returns true if ‖this - other‖∞ < epsilon, otherwise returns false.
  /// Note: the infinity norm ‖this - other‖∞ is simply the maximum of the six
  /// absolute values in (this - other).
  decltype(T() < T()) IsApprox(
      const SpatialQuantity& other,
      double tolerance = 2 * std::numeric_limits<double>::epsilon()) const {
    return IsNearlyEqualWithinAbsoluteTolerance(other, tolerance, tolerance);
  }

  /// Sets all the elements in `this` %SpatialVector to NaN. This is typically
  /// used to quickly detect uninitialized values since NaN will trigger a chain
  /// of invalid computations that can be tracked back to their source.
  void SetNaN() {
    V_.setConstant(std::numeric_limits<
                   typename Eigen::NumTraits<T>::Literal>::quiet_NaN());
  }

  /// Sets both the rotational and translational components of `this`
  /// %SpatialVector to zero.
  SpatialQuantity& SetZero() {
    V_.setZero();
    return get_mutable_derived();
  }

  /// Returns a mutable reference to the underlying storage.
  CoeffsEigenType& get_coeffs() { return V_; }

  /// Returns a constant reference to the underlying storage.
  const CoeffsEigenType& get_coeffs() const { return V_; }

  /// Unary minus operator.
  SpatialQuantity operator-() const { return SpatialQuantity(-get_coeffs()); }

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

  /// Adds two spatial vectors by simply adding their 6 underlying elements.
  /// @param[in] V1_E spatial vector expressed in the same frame E as V2_E.
  /// @param[in] V2_E spatial vector expressed in the same frame E as V1_E.
  /// @note The general utility of this operator+() function is questionable
  /// and it should only be used if you are sure it makes sense.  Please refer
  /// to documentation for the appropriate spatial quantity subclass (e.g.,
  /// SpatialVelocity, SpatialAcceleration, SpatialForce, or SpatialMomentum).
  friend SpatialQuantity operator+(const SpatialQuantity& V1_E,
                                   const SpatialQuantity& V2_E) {
    return SpatialQuantity(V1_E.get_coeffs() + V2_E.get_coeffs());
  }

  /// Subtracts two spatial vectors by simply subtracting their 6 underlying
  /// elements.
  /// @param[in] V1 spatial vector expressed in the same frame as V2.
  /// @param[in] V2 spatial vector expressed in the same frame as V1.
  /// @note The general utility of this operator-() function is questionable
  /// and it should only be used if you are sure it makes sense.  Please refer
  /// to documentation for the appropriate spatial quantity subclass (e.g.,
  /// SpatialVelocity, SpatialAcceleration, SpatialForce, or SpatialMomentum).
  friend SpatialQuantity operator-(const SpatialQuantity& V1,
                                   const SpatialQuantity& V2) {
    return SpatialQuantity(V1.get_coeffs() - V2.get_coeffs());
  }

  /// Multiplication of a spatial vector V from the left by a scalar `s`.
  /// @relates SpatialVector.
  friend SpatialQuantity operator*(const T& s, const SpatialQuantity& V) {
    return SpatialQuantity(V.get_coeffs() * s);
  }

  /// Multiplication of a spatial vector V from the right by a scalar `s`.
  /// @relates SpatialVector.
  friend SpatialQuantity operator*(const SpatialQuantity& V, const T& s) {
    return s * V;  // Multiplication by scalar is commutative.
  }

  /// Expresses a spatial vector in another frame.
  /// @param[in] R_FE RotationMatrix relating a frame F to a frame E.
  /// @param[in] V_E spatial vector expressed in frame E.
  /// @returns V_F spatial vector expressed in frame F, calculated from: <pre>
  ///   V_F.rotational()    = R_FE * V_E.rotational(),
  ///   V_F.translational() = R_FE * V_E.translational()
  /// </pre>
  friend SpatialQuantity operator*(const math::RotationMatrix<T>& R_FE,
                                   const SpatialQuantity& V_E) {
    return SpatialQuantity(R_FE * V_E.rotational(), R_FE * V_E.translational());
  }

  /// Factory to create a _zero_ spatial vector, i.e., a %SpatialVector whose
  /// rotational and translational components are both zero.
  static SpatialQuantity Zero() { return SpatialQuantity{}.SetZero(); }

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
template <template <typename> class SpatialQuantity, typename T>
std::ostream& operator<<(std::ostream& o,
                         const SpatialVector<SpatialQuantity, T>& V) {
  o << "[" << fmt::to_string(V[0]);
  for (int i = 1; i < V.size(); ++i) {
    o << ", " << fmt::to_string(V[i]);
  }
  o << "]ᵀ";  // The "transpose" symbol.
  return o;
}

}  // namespace multibody
}  // namespace drake

// TODO(jwnimmer-tri) Add a real formatter and deprecate the operator<<.
namespace fmt {
template <template <typename> class SpatialQuantity, typename T>
struct formatter<drake::multibody::SpatialVector<SpatialQuantity, T>>
    : drake::ostream_formatter {};
}  // namespace fmt
