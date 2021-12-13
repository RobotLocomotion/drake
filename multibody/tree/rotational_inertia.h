#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Eigenvalues>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_bool.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/common/extract_double.h"
#include "drake/common/symbolic.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace multibody {

/// This class describes the mass distribution (inertia properties) of a
/// body or composite body about a particular point.  Herein, "composite body"
/// means one body or a collection of bodies that are welded together.  In this
/// documentation, "body" and "composite body" are used interchangeably.
///
/// A **rigid** body's mass distribution is described by three quantities:
/// the body's mass; the body's center of mass; and the body's rotational
/// inertia about a particular point. The term **rotational inertia** is used
/// here and by [Jain 2010] to distinguish from a body's **spatial inertia**.
/// In this class, a 3x3 **inertia matrix** I represents a body's rotational
/// inertia about a point and expressed in a frame.  More specifically, `I_BP_E`
/// is the inertia matrix of a body B about-point P and expressed-in frame E
/// (herein frame E's orthogonal unit vectors Ex, Ey, Ez are denoted 𝐱̂, 𝐲̂, 𝐳̂).
/// <pre>
///     | Ixx Ixy Ixz |
/// I = | Ixy Iyy Iyz |
///     | Ixz Iyz Izz |
/// </pre>
/// The moments of inertia Ixx, Iyy, Izz and products of inertia Ixy, Ixz, Iyz
/// are defined in terms of the mass dm of a differential volume of the body.
/// The position of dm from about-point P is xx̂ + yŷ + zẑ = [x, y, z]_E.
/// <pre>
/// Ixx = ∫ (y² + z²) dm
/// Iyy = ∫ (x² + z²) dm
/// Izz = ∫ (x² + y²) dm
/// Ixy = - ∫ x y dm
/// Ixz = - ∫ x z dm
/// Iyz = - ∫ y z dm
/// </pre>
/// We use the negated convention for products of inertia, so that I serves
/// to relate angular velocity ω and angular momentum h via `h = I ⋅ ω`.
/// Ensure your products of inertia follow this negative sign convention.
///
/// The 3x3 inertia matrix is symmetric and its diagonal elements (moments of
/// inertia) and off-diagonal elements (products of inertia) are associated
/// with a body (or composite body) S, an about-point P, and an expressed-in
/// frame E (𝐱̂, 𝐲̂, 𝐳̂̂).  A rotational inertia is ill-defined unless there is a
/// body S, about-point P, and expressed-in frame E. The user of this class is
/// responsible for tracking the body S, about-point P and expressed-in frame E
/// (none of these are stored in this class).
///
/// @note This class does not store the about-point nor the expressed-in frame,
/// nor does this class help enforce consistency of the about-point or
/// expressed-in frame. To help users of this class track the about-point and
/// expressed-in frame, we strongly recommend the following notation.
///
/// @note In typeset material, use the symbol @f$ [I^{S/P}]_E @f$ to represent
/// the rotational inertia (inertia matrix) of a body (or composite body) S
/// about-point P, expressed in frame E. In code and comments, use the monogram
/// notation `I_SP_E` (e.g., as described in @ref multibody_spatial_inertia).
/// If the about-point P is fixed to a body B, the point is named @f$ B_P @f$
/// and this appears in code/comments as `Bp`.  Examples: `I_BBp_E` is rigid
/// body B's rotational inertia about-point Bp expressed-in frame E; I_BBo_E is
/// B's rotational inertia about-point `Bo` (body B's origin) expressed-in frame
/// E; and I_BBcm_E is B's inertia matrix about-point `Bcm` (B's center of
/// mass) expressed-in frame E.
///
/// @note The rotational inertia (inertia matrix) can be re-expressed in terms
/// of a special frame whose orthogonal unit vectors are parallel to **principal
/// axes of inertia** so that the inertia matrix is diagonalized with elements
/// called **principal moments of inertia**.
///
/// @note The formal definition of the inertia matrix @f$ I^{S/P} @f$ of a
/// system S about a point P follows the definition of the inertia dyadic 𝐈 of
/// S about P, which begins by modeling S with n particles S₁ ... Sₙ (e.g., 12
/// grams of carbon can be modeled with n = 6.02 * 10²³ molecules/particles).
/// The inertia dyadic 𝐈₁ of one particle S₁ about point P is defined [Kane,
/// 1985] in terms of m₁ (mass of S₁), ᴾ𝐩ˢ¹ (position vector from P to S₁), and
/// the unit dyadic 𝐔 which is defined by the property 𝐔 ⋅ 𝐯 = 𝐯 where 𝐯 is
/// is any vector (this definition of 𝐔 is analogous to defining the identity
/// matrix by the property 𝑰𝒅𝒆𝒏𝒕𝒊𝒕𝒚𝑴𝒂𝒕𝒓𝒊𝒙 * 𝒂𝒏𝒚𝑴𝒂𝒕𝒓𝒊𝒙 = 𝒂𝒏𝒚𝑴𝒂𝒕𝒓𝒊𝒙).
/// <pre>
///     𝐈₁ = m₁ * [𝐔 * (ᴾ𝐩ˢ¹ ⋅ ᴾ𝐩ˢ¹)  -  ᴾ𝐩ˢ¹ * ᴾ𝐩ˢ¹]
/// </pre>
/// Note: The vector dot-product (⋅) above produces a scalar whereas the vector
/// multiply (*) produces a dyadic which is a 2nd-order tensor (ᴾ𝐩ˢ¹ * ᴾ𝐩ˢ¹ is
/// similar to the matrix outer-product of a 3x1 matrix multiplied by a 1x3
/// matrix). An example inertia dyadic for a single particle is shown further
/// below.  The inertia dyadic 𝐈 of the entire system S is defined by summing
/// the inertia dyadic of each particle Sᵢ about P (i = 1, ... n), i.e.,
/// <pre>
///     𝐈 = 𝐈₁ + 𝐈₂ + ... 𝐈ₙ
/// </pre>
/// The elements of the inertia matrix @f$ [I^{S/P}]_E @f$ expressed in frame E
/// (in terms of orthogonal unit vectors 𝐱̂, 𝐲̂, 𝐳̂̂) are found by pre-dot
/// multiplying and post-dot multiplying 𝐈 with appropriate unit vectors.
/// <pre>
///    Ixx = 𝐱̂ ⋅ 𝐈 ⋅ 𝐱̂     Ixy = 𝐱̂ ⋅ 𝐈 ⋅ 𝐲̂      Ixz = 𝐱̂ ⋅ 𝐈 ⋅ 𝐳̂̂
///    Iyx = 𝐲̂ ⋅ 𝐈 ⋅ 𝐱̂     Iyy = 𝐲̂ ⋅ 𝐈 ⋅ 𝐲̂      Iyz = 𝐲̂ ⋅ 𝐈 ⋅ 𝐳̂̂
///    Izx = 𝐳̂̂ ⋅ 𝐈 ⋅ 𝐱̂     Izy = 𝐳̂̂ ⋅ 𝐈 ⋅ 𝐲̂      Izz = 𝐳̂̂ ⋅ 𝐈 ⋅ 𝐳̂̂
/// </pre>
/// The inertia dyadic 𝐈ᴮ of a rigid body B about Bcm (B's center of mass) is
/// related to various dynamic quantities. For example, B's angular momentum 𝐇
/// about Bcm in a frame N and B's kinetic energy KE in N relate to 𝐈ᴮ by
/// <pre>
///    𝐇 = 𝐈ᴮ ⋅ 𝛚
///    KE = 1/2 𝛚 ⋅ 𝐈ᴮ ⋅ 𝛚  +  1/2 mᴮ 𝐯 ⋅ 𝐯
/// </pre>
/// where 𝛚 is B's angular velocity in N, 𝐯 is Bcm's translational velocity in
/// N, and mᴮ is B's mass.  When frame N happens to be a Newtonian frame (also
/// called an inertial frame or non-rotating/non-accelerating frame), the moment
/// 𝐓 of all forces on B about Bcm relates to 𝐈ᴮ and 𝛂 (B's angular
/// acceleration in N) by Euler's rigid body equation as
/// <pre>
///    𝐓 = 𝐈ᴮ ⋅ 𝛂  +  𝛚 × 𝐈ᴮ ⋅ 𝛚
/// </pre>
/// Example: For a particle Q of mass m whose position vector from a point O is
/// written in terms of right-handed orthogonal unit vectors 𝐱̂, 𝐲̂, 𝐳̂ (below),
/// the inertia dyadic 𝐈 of particle Q about point O is defined and calculated
/// <pre>
///     𝐩 = x 𝐱̂  +  y 𝐲̂                               (given)
///     𝐈 = m * [𝐔 * (𝐩 ⋅ 𝐩)  -  𝐩 * 𝐩]              (definition)
///       = m * [𝐔 * (x² + y²)  -  (x𝐱̂ + y𝐲̂̂) * (x𝐱̂ + y𝐲̂)
///       = m * [(𝐱̂𝐱̂ + 𝐲̂𝐲̂ + 𝐳̂𝐳̂) * (x² + y²) - (x²𝐱̂𝐱̂ + xy𝐱̂𝐲̂̂ + xy𝐲̂̂𝐱̂ + y²𝐲̂̂𝐲̂̂)]
///       = m * [y²𝐱̂𝐱̂ + x²𝐲̂𝐲̂ + (x² + y²)𝐳̂𝐳̂ - xy𝐱̂𝐲̂̂ - xy𝐲̂̂𝐱̂]
/// </pre>
/// which means the inertia matrix for particle Q about point O for 𝐱̂, 𝐲̂, 𝐳̂ is
/// <pre>
///     |  m y²     -m x y         0     |
/// I = | -m x y     m x²          0     |
///     |    0         0     m (x² + y²) |
/// </pre>
/// [Kane, 1985] pg. 68. "Dynamics: Theory and Applications," McGraw-Hill Co.,
/// New York, 1985 (with D. A. Levinson).  Available for free .pdf download:
/// https://ecommons.cornell.edu/handle/1813/637
///
/// @note Several methods in this class throw a std::exception for invalid
/// rotational inertia operations in debug releases only.  This provides speed
/// in a release build while facilitating debugging in debug builds.
/// In addition, these validity tests are only performed for scalar types for
/// which drake::scalar_predicate<T>::is_bool is `true`. For instance, validity
/// checks are not performed when T is symbolic::Expression.
///
/// Various methods in this class require numerical (not symbolic) data types.
///
/// @tparam_default_scalar
template <typename T>
class RotationalInertia {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RotationalInertia)

  /// Constructs a rotational inertia that has all its moments/products of
  /// inertia equal to NaN (helps quickly detect uninitialized values).
  RotationalInertia() {}

  /// Creates a rotational inertia with moments of inertia `Ixx`, `Iyy`, `Izz`,
  /// and with each product of inertia set to zero.
  /// @throws std::exception for Debug builds if not CouldBePhysicallyValid().
  RotationalInertia(const T& Ixx, const T& Iyy, const T& Izz)
      : RotationalInertia(Ixx, Iyy, Izz, 0.0, 0.0, 0.0) {}

  /// Creates a rotational inertia with moments of inertia `Ixx`, `Iyy`, `Izz`,
  /// and with products of inertia `Ixy`, `Ixz`, `Iyz`.
  /// @throws std::exception for Debug builds if not CouldBePhysicallyValid().
  RotationalInertia(const T& Ixx, const T& Iyy, const T& Izz,
                    const T& Ixy, const T& Ixz, const T& Iyz) {
    set_moments_and_products_no_validity_check(Ixx, Iyy, Izz,
                                               Ixy, Ixz, Iyz);
    DRAKE_ASSERT_VOID(ThrowIfNotPhysicallyValid(__func__));
  }

  /// Constructs a rotational inertia for a particle Q of mass `mass`, whose
  /// position vector from about-point P is p_PQ_E (E is expressed-in frame).
  /// This std::logic_error exception only occurs if `mass` < 0.
  /// @param mass The mass of particle Q.
  /// @param p_PQ_E Position from about-point P to Q, expressed-in frame E.
  /// @retval I_QP_E, Q's rotational inertia about-point P expressed-in frame E.
  /// @remark Negating the position vector p_PQ_E has no affect on the result.
  /// @throws std::exception for Debug builds if not CouldBePhysicallyValid().
  RotationalInertia(const T& mass, const Vector3<T>& p_PQ_E)
      : RotationalInertia(mass * p_PQ_E, p_PQ_E) {}

  /// Constructs a rotational inertia with equal moments of inertia along its
  /// diagonal and with each product of inertia set to zero. This factory
  /// is useful for the rotational inertia of a uniform-density sphere or cube.
  /// In debug builds, throws std::exception if I_triaxial is negative/NaN.
  // TODO(mitiguy) Per issue #6139  Update to MakeTriaxiallySymmetric.
  static RotationalInertia<T> TriaxiallySymmetric(const T& I_triaxial) {
    return RotationalInertia(I_triaxial, I_triaxial, I_triaxial, 0.0, 0.0, 0.0);
  }

  /// (Internal use only) Creates a rotational inertia with moments of inertia
  /// Ixx, Iyy, Izz, and with products of inertia Ixy, Ixz, Iyz.
  /// @param[in] Ixx, Iyy, Izz Moments of inertia.
  /// @param[in] Ixy, Ixz, Iyz Products of inertia.
  /// @param[in] skip_validity_check If set to false, the rotational inertia is
  /// checked via CouldBePhysicallyValid() to ensure it is physically valid.
  /// If set to true (not generally recommended), the check is skipped (which
  /// reduces some computational cost). The default value is false.
  /// @throws std::exception if skip_validity_check is false and
  /// CouldBePhysicallyValid() fails.
  static RotationalInertia<T> MakeFromMomentsAndProductsOfInertia(
      const T& Ixx, const T& Iyy, const T& Izz,
      const T& Ixy, const T& Ixz, const T& Iyz,
      bool skip_validity_check = false) {
    RotationalInertia<T> I;
    I.set_moments_and_products_no_validity_check(Ixx, Iyy, Izz,
                                                 Ixy, Ixz, Iyz);
    if (!skip_validity_check) I.ThrowIfNotPhysicallyValid(__func__);
    return I;
  }

  /// For consistency with Eigen's API, the rows() method returns 3.
  int rows() const { return 3; }

  /// For consistency with Eigen's API, the cols() method returns 3.
  int cols() const { return 3; }

  /// Returns 3-element vector with moments of inertia [Ixx, Iyy, Izz].
  Vector3<T> get_moments() const { return I_SP_E_.diagonal(); }

  /// Returns 3-element vector with products of inertia [Ixy, Ixz, Iyz].
  Vector3<T> get_products() const {
    // Note: Products of inertia are stored in lower-triangular part of matrix.
    // Note: The three upper off-diagonal matrix elements remain equal to NaN.
    static_assert(is_lower_triangular_order(1, 0), "Invalid indices");
    static_assert(is_lower_triangular_order(2, 0), "Invalid indices");
    static_assert(is_lower_triangular_order(2, 1), "Invalid indices");
    return Vector3<T>(I_SP_E_(1, 0), I_SP_E_(2, 0), I_SP_E_(2, 1));
  }

  /// Returns a rotational inertia's trace (i.e., Ixx + Iyy + Izz, the sum of
  /// the diagonal elements of the inertia matrix).  The trace happens to be
  /// invariant to its expressed-in frame (i.e., the trace does not depend
  /// on the frame in which it is expressed).  The trace is useful because the
  /// largest moment of inertia Imax has range: trace / 3 <= Imax <= trace / 2,
  /// and the largest possible product of inertia must be <= Imax / 2.
  /// Hence, trace / 3 and trace / 2 give a lower and upper bound on the largest
  /// possible element that can be in a valid rotational inertia.
  T Trace() const { return I_SP_E_.trace(); }

  /// Returns the maximum possible moment of inertia for `this` rotational
  /// inertia about-point P for any expressed-in frame E.
  /// @remark The maximum moment Imax has range: trace / 3 <= Imax <= trace / 2.
  /// @see Trace()
  T CalcMaximumPossibleMomentOfInertia() const {
    using std::abs;
    return 0.5 * abs(Trace());
  }

  /// Const access to the `(i, j)` element of this rotational inertia.
  /// @remark A mutable version of operator() is intentionally absent so as to
  /// prevent an end-user from directly setting elements.  This prevents the
  /// creation of a non-physical (or non-symmetric) rotational inertia.
  const T& operator()(int i, int j) const {
    // Overwrites local copies of i and j.
    check_and_swap(&i, &j);
    DRAKE_ASSERT(is_lower_triangular_order(i, j));
    return I_SP_E_(i, j);
  }

  /// Gets a full 3x3 matrix copy of this rotational inertia.  The returned copy
  /// is symmetric and includes both lower and upper parts of the matrix.
  Matrix3<T> CopyToFullMatrix3() const { return get_symmetric_matrix_view(); }

  /// Compares `this` rotational inertia to `other` rotional inertia within the
  /// specified `precision` (which is a dimensionless number specifying
  /// the relative precision to which the comparison is performed).
  /// Denoting `I_maxA` as the largest element value that can appear in a valid
  /// `this` rotational inertia (independent of the expressed-in frame E) and
  /// denoting `I_maxB` as the largest element value that can appear in a valid
  /// `other` rotational inertia (independent of the expressed-in frame E),
  /// `this` and `other` are considered nearly equal to each other, if:
  ///  ‖this - other‖∞  <  precision * min(I_maxA, I_maxB)
  ///
  /// @param other Rotational inertia to compare with `this` rotational inertia.
  /// @param  precision is a dimensionless real positive number that is usually
  ///         based on two factors, namely expected accuracy of moments/products
  ///         of inertia (e.g., from end-user or CAD) and/or machine-precision.
  /// @return `true` if the absolute value of each moment/product of inertia
  ///          in `this` is within `epsilon` of the corresponding moment/
  ///          product absolute value in `other`.  Otherwise returns `false`.
  /// @note: This method only works if all moments of inertia with scalar type T
  ///    in `this` and `other` can be converted to a double (discarding
  ///    supplemental scalar data such as derivatives of an AutoDiffScalar).
  ///    It fails at runtime if type T cannot be converted to `double`.
  boolean<T> IsNearlyEqualTo(const RotationalInertia& other,
                             double precision) const {
    using std::min;
    const T I_maxA = CalcMaximumPossibleMomentOfInertia();
    const T I_maxB = other.CalcMaximumPossibleMomentOfInertia();
    const T I_test = min(I_maxA, I_maxB);
    const T epsilon = precision * I_test;
    return IsApproxMomentsAndProducts(other, epsilon);
  }

  /// Adds a rotational inertia `I_BP_E` to `this` rotational inertia.
  /// This method requires both rotational inertias (`I_BP_E` and `this`)
  /// to have the same about-point P and the same expressed-in frame E.
  /// The += operator updates `this` so `I_BP_E` is added to `this`.
  /// @param I_BP_E Rotational inertia of a body (or composite body) B to
  ///        be added to `this` rotational inertia.  `I_BP_E` and `this`
  ///        must have the same about-point P and expressed-in frame E.
  /// @return A reference to `this` rotational inertia. `this` changes
  ///         since rotational inertia `I_BP_E` has been added to it.
  /// @see operator+().
  // TODO(Mitiguy) Issue #6145, add direct unit test for this method.
  RotationalInertia<T>& operator+=(const RotationalInertia<T>& I_BP_E) {
    this->get_mutable_triangular_view() += I_BP_E.get_matrix();
    return *this;
  }

  /// Adds a rotational inertia `I_BP_E` to `this` rotational inertia.
  /// This method requires both rotational inertias (`I_BP_E` and `this`)
  /// to have the same about-point P and the same expressed-in frame E.
  /// @param I_BP_E Rotational inertia of a body (or composite body) B to
  ///        be added to `this` rotational inertia.  `I_BP_E` and `this`
  ///        must have the same about-point P and expressed-in frame E.
  /// @return The sum of `this` rotational inertia and `I_BP_E`.
  /// @see operator+=().
  RotationalInertia<T> operator+(const RotationalInertia<T>& I_BP_E) const {
    return RotationalInertia(*this) += I_BP_E;
  }

  /// Subtracts a rotational inertia `I_BP_E` from `this` rotational inertia.
  /// This method requires both rotational inertias (`I_BP_E` and `this`)
  /// to have the same about-point P and the same expressed-in frame E.
  /// The -= operator updates `this` so `I_BP_E` is subtracted from `this`.
  /// @param I_BP_E Rotational inertia of a body (or composite body) B to
  ///        be subtracted from `this` rotational inertia. `I_BP_E` and `this`
  ///        must have the same about-point P and expressed-in frame E.
  /// @return A reference to `this` rotational inertia. `this` changes
  ///         since rotational inertia `I_BP_E` has been subtracted from it.
  /// @throws std::exception for Debug builds if not CouldBePhysicallyValid().
  /// @see operator-().
  /// @note This subtract operator is useful for computing rotational inertia
  /// of a body with a hole.  First the rotational inertia of a fully solid
  /// body S (without the hole) is calculated, then the rotational inertia of
  /// the hole (treated as a massive solid body B) is calculated. The rotational
  /// inertia of a composite body C (comprised of S and -B) is computed by
  /// subtracting B's rotational inertia from S's rotational inertia.
  RotationalInertia<T>& operator-=(const RotationalInertia<T>& I_BP_E) {
    MinusEqualsUnchecked(I_BP_E);
    DRAKE_ASSERT_VOID(ThrowIfNotPhysicallyValid(__func__));
    return *this;
  }

  /// Subtracts a rotational inertia `I_BP_E` from `this` rotational inertia.
  /// This method requires both rotational inertias (`I_BP_E` and `this`)
  /// to have the same about-point P and the same expressed-in frame E.
  /// @param I_BP_E Rotational inertia of a body (or composite body) B to
  ///        be subtracted from `this` rotational inertia. `I_BP_E` and `this`
  ///        must have the same about-point P and expressed-in frame E.
  /// @return The subtraction of `I_BP_E` from `this` rotational inertia.
  /// @throws std::exception for Debug builds if not CouldBePhysicallyValid().
  /// @see operator-=().
  /// @warning See warning and documentation for operator-=().
  RotationalInertia<T> operator-(const RotationalInertia<T>& I_BP_E) const {
    return RotationalInertia(*this) -= I_BP_E;
  }

  /// Multiplies `this` rotational inertia by a nonnegative scalar (>= 0).
  /// In debug builds, throws std::exception if `nonnegative_scalar` < 0.
  /// @param nonnegative_scalar Nonnegative scalar which multiplies `this`.
  /// @return A reference to `this` rotational inertia. `this` changes
  ///         since `this` has been multiplied by `nonnegative_scalar`.
  /// @see operator*(), operator*(const T&, const RotationalInertia<T>&).
  RotationalInertia<T>& operator*=(const T& nonnegative_scalar) {
    DRAKE_ASSERT_VOID(
        this->ThrowIfMultiplyByNegativeScalar(nonnegative_scalar));
    this->get_mutable_triangular_view() *= nonnegative_scalar;
    return *this;
  }

  /// Multiplies `this` rotational inertia by a nonnegative scalar (>= 0).
  /// In debug builds, throws std::exception if `nonnegative_scalar` < 0.
  /// @param nonnegative_scalar Nonnegative scalar which multiplies `this`.
  /// @return `this` rotational inertia multiplied by `nonnegative_scalar`.
  /// @see operator*=(), operator*(const T&, const RotationalInertia<T>&)
  RotationalInertia<T> operator*(const T& nonnegative_scalar) const {
    return RotationalInertia(*this) *= nonnegative_scalar;
  }

  /// Multiplies a nonnegative scalar (>= 0) by the rotational inertia `I_BP_E`.
  /// In debug builds, throws std::exception if `nonnegative_scalar` < 0.
  /// @param nonnegative_scalar Nonnegative scalar which multiplies `I_BP_E`.
  /// @return `nonnegative_scalar` multiplied by rotational inertia `I_BP_E`.
  /// @see operator*=(), operator*()
  friend RotationalInertia<T> operator*(const T& nonnegative_scalar,
                                        const RotationalInertia<T>& I_BP_E) {
    /// Multiplication of a scalar with a rotational matrix is commutative.
    return RotationalInertia(I_BP_E) *= nonnegative_scalar;
  }

  /// Multiplies `this` rotational inertia about-point P, expressed-in frame E
  /// by the vector w_E (which *must* also have the same expressed-in frame E).
  /// @note This calculation is equivalent to regarding `this` rotational
  ///       inertia as an inertia dyadic and dot-multiplying it by w_E.
  /// @param w_E Vector to post-multiply with `this` rotational inertia.
  /// @return The Vector that results from multiplying `this` by `w_E`.
  // TODO(Mitiguy) Issue #6145, add direct unit test for this method.
  Vector3<T> operator*(const Vector3<T>& w_E) const {
    return Vector3<T>(get_symmetric_matrix_view() * w_E);
  }

  /// Divides `this` rotational inertia by a positive scalar (> 0).
  /// In debug builds, throws std::exception if `positive_scalar` <= 0.
  /// @param positive_scalar Positive scalar (> 0) which divides `this`.
  /// @return A reference to `this` rotational inertia. `this` changes
  ///         since `this` has been divided by `positive_scalar`.
  /// @see operator/().
  RotationalInertia<T>& operator/=(const T& positive_scalar) {
    DRAKE_ASSERT_VOID(ThrowIfDivideByZeroOrNegativeScalar(positive_scalar));
    this->get_mutable_triangular_view() /= positive_scalar;
    return *this;
  }

  /// Divides `this` rotational inertia by a positive scalar(> 0).
  /// In debug builds, throws std::exception if `positive_scalar` <= 0.
  /// @param positive_scalar Positive scalar (> 0) which divides `this`.
  /// @return `this` rotational inertia divided by `positive_scalar`.
  /// @see operator/=().
  // TODO(Mitiguy) Issue #6145, add direct unit test for this method.
  RotationalInertia<T> operator/(const T& positive_scalar) const {
    return RotationalInertia(*this) /= positive_scalar;
  }

  /// (Internal use only) Multiplies a rotational inertia by a scalar.
  /// @param[in] s Scalar which multiplies `this`.
  /// @return `this` rotational inertia multiplied by `s`.
  /// @see operator*(const T&, const RotationalInertia<T>&).
  /// @note This method works even if `s` is negative or `this` is invalid. This
  /// method is useful for error messages associated with an invalid inertia.
  RotationalInertia<T> MultiplyByScalarSkipValidityCheck(const T& s) const {
    RotationalInertia<T> I(*this);  // Mutable copy of `this`.
    I.get_mutable_triangular_view() *= s;
    return I;
  }

  /// Sets `this` rotational inertia so all its elements are equal to NaN.
  /// This helps quickly detect uninitialized moments/products of inertia.
  void SetToNaN() {
    I_SP_E_.setConstant(std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN());
  }

  /// Sets `this` rotational inertia so all its moments/products of inertia
  /// are zero, e.g., for convenient initialization before a computation or
  /// for inertia calculations involving a particle (point-mass).
  /// Note: Real 3D massive physical objects have non-zero moments of inertia.
  void SetZero() {
    // Only set the lower-triangular part of this symmetric matrix to zero.
    // The three upper off-diagonal products of inertia should be/remain NaN.
    I_SP_E_.template triangularView<Eigen::Lower>() = Matrix3<T>::Zero();
  }

  /// Returns `true` if any moment/product in `this` rotational inertia is NaN.
  /// Otherwise returns `false`.
  boolean<T> IsNaN() const {
    using std::isnan;
    // Only check the lower-triangular part of this symmetric matrix for NaN.
    // The three upper off-diagonal products of inertia should be/remain NaN.
    static_assert(is_lower_triangular_order(0, 0), "Invalid indices");
    static_assert(is_lower_triangular_order(1, 0), "Invalid indices");
    static_assert(is_lower_triangular_order(2, 0), "Invalid indices");
    static_assert(is_lower_triangular_order(1, 1), "Invalid indices");
    static_assert(is_lower_triangular_order(2, 1), "Invalid indices");
    static_assert(is_lower_triangular_order(2, 2), "Invalid indices");
    return isnan(I_SP_E_(0, 0)) ||
        isnan(I_SP_E_(1, 0)) || isnan(I_SP_E_(1, 1)) ||
        isnan(I_SP_E_(2, 0)) || isnan(I_SP_E_(2, 1)) || isnan(I_SP_E_(2, 2));
  }

  /// Returns a new %RotationalInertia object templated on `Scalar` initialized
  /// from the values of `this` rotational inertia's entries.
  ///
  /// @tparam Scalar The scalar type on which the new rotational inertia will
  /// be templated.
  ///
  /// @note `RotationalInertia<From>::cast<To>()` creates a new
  /// `RotationalInertia<To>` from a `RotationalInertia<From>` but only if
  /// type `To` is constructible from type `From`.
  /// This cast method works in accordance with Eigen's cast method for Eigen's
  /// %Matrix3 that underlies this %RotationalInertia.  For example, Eigen
  /// currently allows cast from type double to AutoDiffXd, but not vice-versa.
  template <typename Scalar>
  RotationalInertia<Scalar> cast() const {
    // Skip validity check since this inertia is already valid.
    return RotationalInertia<Scalar>(I_SP_E_.template cast<Scalar>(), true);
  }

  /// This method takes `this` rotational inertia about-point P, expressed-in
  /// frame E, and computes its principal moments of inertia about-point P, but
  /// expressed-in a frame aligned with the principal axes.
  ///
  /// @note: This method only works for a rotational inertia with scalar type T
  ///        that can be converted to a double (discarding any supplemental
  ///        scalar data such as derivatives of an AutoDiffScalar).
  ///
  /// @retval principal_moments The vector of principal moments of inertia
  ///                           `[Ixx Iyy Izz]` sorted in ascending order.
  /// @throws std::exception if eigenvalue solver fails or if scalar type T
  ///         cannot be converted to a double.
  Vector3<double> CalcPrincipalMomentsOfInertia() const {
    // Notes:
    //   1. Eigen's SelfAdjointEigenSolver does not compile for AutoDiffScalar.
    //      Therefore, convert `this` to a local copy of type Matrix3<double>.
    //   2. Eigen's SelfAdjointEigenSolver only uses the lower-triangular part
    //      of this symmetric matrix.
    static_assert(is_lower_triangular_order(1, 0), "Invalid indices");
    static_assert(is_lower_triangular_order(2, 0), "Invalid indices");
    static_assert(is_lower_triangular_order(2, 1), "Invalid indices");
    Matrix3<double> I_double = Matrix3<double>::Constant(NAN);
    I_double(0, 0) = ExtractDoubleOrThrow(I_SP_E_(0, 0));
    I_double(1, 1) = ExtractDoubleOrThrow(I_SP_E_(1, 1));
    I_double(2, 2) = ExtractDoubleOrThrow(I_SP_E_(2, 2));
    I_double(1, 0) = ExtractDoubleOrThrow(I_SP_E_(1, 0));
    I_double(2, 0) = ExtractDoubleOrThrow(I_SP_E_(2, 0));
    I_double(2, 1) = ExtractDoubleOrThrow(I_SP_E_(2, 1));

    // If all products of inertia are zero, no need to calculate eigenvalues.
    // The eigenvalues are the diagonal elements.  Sort them in ascending order.
    const bool is_diagonal = (I_double(1, 0) == 0 && I_double(2, 0) == 0 &&
                              I_double(2, 1) == 0);
    if (is_diagonal) {
      Vector3<double> moments(I_double(0, 0), I_double(1, 1), I_double(2, 2));
      std::sort(moments.data(), moments.data() + moments.size());
      return moments;
    }

    Eigen::SelfAdjointEigenSolver<Matrix3<double>> solver(
        I_double, Eigen::EigenvaluesOnly);
    if (solver.info() != Eigen::Success) {
      throw std::runtime_error(
          "Error: In RotationalInertia::CalcPrincipalMomentsOfInertia()."
          " Solver failed while computing eigenvalues of the inertia matrix.");
    }
    return solver.eigenvalues();
  }

  /// Performs several necessary checks to verify whether `this` rotational
  /// inertia *could* be physically valid, including:
  ///
  /// - No NaN moments or products of inertia.
  /// - Ixx, Iyy, Izz and principal moments are all non-negative.
  /// - Ixx, Iyy  Izz and principal moments satisfy the triangle inequality:
  ///   - `Ixx + Iyy >= Izz`
  ///   - `Ixx + Izz >= Iyy`
  ///   - `Iyy + Izz >= Ixx`
  ///
  /// @warning These checks are necessary (but NOT sufficient) conditions for a
  /// rotational inertia to be physically valid.  The sufficient condition
  /// requires a rotational inertia to satisfy the above checks *after* `this`
  /// is shifted to the center of mass, i.e., the sufficient condition requires
  /// calling CouldBePhysicallyValid() when the about-point is Bcm (the body's
  /// center of mass).  Note: this class does not know its about-point or its
  /// center of mass location.
  ///
  /// @return `true` for a plausible rotational inertia passing the above
  ///          necessary but insufficient checks and `false` otherwise.
  /// @throws std::exception if principal moments of inertia cannot be
  ///         calculated (eigenvalue solver) or if scalar type T cannot be
  ///         converted to a double.
  boolean<T> CouldBePhysicallyValid() const {
    // To check the validity of rotational inertia use an epsilon value that is
    // a number related to machine precision multiplied by the largest possible
    // element that can appear in a valid `this` rotational inertia.  Note: The
    // largest product of inertia is at most half the largest moment of inertia.
    using std::max;
    const double precision = 10 * std::numeric_limits<double>::epsilon();
    const T max_possible_inertia_moment = CalcMaximumPossibleMomentOfInertia();

    // In order to avoid false negatives for inertias close to zero we use, in
    // addition to a relative tolerance of "precision", an absolute tolerance
    // equal to "precision" as well.
    const T epsilon = precision * max(1.0, max_possible_inertia_moment);

    // Calculate principal moments of inertia p and then test these principal
    // moments to be mostly non-negative and also satisfy triangle inequality.
    const Vector3<double> p = CalcPrincipalMomentsOfInertia();

    return !IsNaN() &&
        AreMomentsOfInertiaNearPositiveAndSatisfyTriangleInequality(
            p(0), p(1), p(2), epsilon);
  }

  /// Re-expresses `this` rotational inertia `I_BP_E` in place to `I_BP_A`.
  /// In other words, starts with `this` rotational inertia of a body (or
  /// composite body) B about-point P expressed-in frame E and re-expresses
  /// to B's rotational inertia about-point P expressed-in frame A. More
  /// concisely, we compute `I_BP_A = R_AE * I_BP_E * (R_AE)ᵀ`.
  ///
  /// @param[in] R_AE RotationMatrix relating frames A and E.
  /// @return A reference to `this` rotational inertia about-point P, but
  ///         with `this` now expressed in frame A (instead of frame E).
  /// @throws std::exception for Debug builds if the rotational inertia that
  /// is re-expressed-in frame A violates CouldBePhysicallyValid().
  /// @see ReExpress().
  RotationalInertia<T>& ReExpressInPlace(const math::RotationMatrix<T>& R_AE);

  /// Re-expresses `this` rotational inertia `I_BP_E` to `I_BP_A`
  /// i.e., re-expresses body B's rotational inertia from frame E to frame A.
  /// @param[in] R_AE RotationMatrix relating frames A and E.
  /// @retval I_BP_A Rotational inertia of B about-point P expressed-in frame A.
  /// @throws std::exception for Debug builds if the rotational inertia that
  /// is re-expressed-in frame A violates CouldBePhysicallyValid().
  /// @see ReExpressInPlace()
  RotationalInertia<T> ReExpress(const math::RotationMatrix<T>& R_AE) const
      __attribute__((warn_unused_result)) {
    return RotationalInertia(*this).ReExpressInPlace(R_AE);
  }

  /// @name Shift methods
  ///  Each shift method shifts a body's rotational inertia from one about-point
  ///  to another about-point. The expressed-in frame is unchanged.
  ///
  /// In-place methods (`this` changes)      | Const methods
  /// ---------------------------------------|--------------------------------
  /// ShiftFromCenterOfMassInPlace           | ShiftFromCenterOfMass
  /// ShiftToCenterOfMassInPlace             | ShiftToCenterOfMass
  /// ShiftToThenAwayFromCenterOfMassInPlace | ShiftToThenAwayFromCenterOfMass
  ///@{

  /// Shifts `this` rotational inertia for a body (or composite body) B
  /// from about-point Bcm (B's center of mass) to about-point Q.
  /// I.e., shifts `I_BBcm_E` to `I_BQ_E` (both are expressed-in frame E).
  /// @param mass The mass of body (or composite body) B.
  /// @param p_BcmQ_E Position vector from Bcm to Q, expressed-in frame E.
  /// @return A reference to `this` rotational inertia expressed-in frame E,
  ///         but with `this` shifted from about-point Bcm to about-point Q.
  ///         i.e., returns I_BQ_E,  B's rotational inertia about-point Bcm
  ///         expressed-in frame E.
  /// @throws std::exception for Debug builds if the rotational inertia that
  /// is shifted to about-point Q violates CouldBePhysicallyValid().
  /// @remark Negating the position vector p_BcmQ_E has no affect on the result.
  RotationalInertia<T>& ShiftFromCenterOfMassInPlace(
      const T& mass,
      const Vector3<T>& p_BcmQ_E) {
    *this += RotationalInertia(mass, p_BcmQ_E);
    return *this;
  }

  /// Calculates the rotational inertia that results from shifting `this`
  /// rotational inertia for a body (or composite body) B
  /// from about-point Bcm (B's center of mass) to about-point Q.
  /// I.e., shifts `I_BBcm_E` to `I_BQ_E` (both are expressed-in frame E).
  /// @param mass The mass of body (or composite body) B.
  /// @param p_BcmQ_E Position vector from Bcm to Q, expressed-in frame E.
  /// @retval I_BQ_E B's rotational inertia about-point Q expressed-in frame E.
  /// @throws std::exception for Debug builds if the rotational inertia that
  /// is shifted to about-point Q violates CouldBePhysicallyValid().
  /// @remark Negating the position vector p_BcmQ_E has no affect on the result.
  RotationalInertia<T> ShiftFromCenterOfMass(const T& mass,
                                             const Vector3<T>& p_BcmQ_E) const
                                           __attribute__((warn_unused_result)) {
    return RotationalInertia(*this).
           ShiftFromCenterOfMassInPlace(mass, p_BcmQ_E);
  }

  /// Shifts `this` rotational inertia for a body (or composite body) B
  /// from about-point Q to about-point `Bcm` (B's center of mass).
  /// I.e., shifts `I_BQ_E` to `I_BBcm_E` (both are expressed-in frame E).
  /// @param mass The mass of body (or composite body) B.
  /// @param p_QBcm_E Position vector from Q to `Bcm`, expressed-in frame E.
  /// @return A reference to `this` rotational inertia expressed-in frame E,
  ///         but with `this` shifted from about-point Q to about-point `Bcm`,
  ///         i.e., returns `I_BBcm_E`, B's rotational inertia about-point `Bcm`
  ///         expressed-in frame E.
  /// @throws std::exception for Debug builds if the rotational inertia that
  /// is shifted to about-point `Bcm` violates CouldBePhysicallyValid().
  /// @remark Negating the position vector `p_QBcm_E` has no affect on the
  /// result.
  RotationalInertia<T>& ShiftToCenterOfMassInPlace(const T& mass,
                                                   const Vector3<T>& p_QBcm_E) {
    *this -= RotationalInertia(mass, p_QBcm_E);
    return *this;
  }

  /// Calculates the rotational inertia that results from shifting `this`
  /// rotational inertia for a body (or composite body) B
  /// from about-point Q to about-point `Bcm` (B's center of mass).
  /// I.e., shifts `I_BQ_E` to `I_BBcm_E` (both are expressed-in frame E).
  /// @param mass The mass of body (or composite body) B.
  /// @param p_QBcm_E Position vector from Q to `Bcm`, expressed-in frame E.
  /// @retval I_BBcm_E B's rotational inertia about-point `Bcm` expressed-in
  /// frame E.
  /// @throws std::exception for Debug builds if the rotational inertia that
  /// is shifted to about-point `Bcm` violates CouldBePhysicallyValid().
  /// @remark Negating the position vector `p_QBcm_E` has no affect on the
  /// result.
  RotationalInertia<T> ShiftToCenterOfMass(const T& mass,
                                           const Vector3<T>& p_QBcm_E) const
                                           __attribute__((warn_unused_result)) {
    return RotationalInertia(*this).ShiftToCenterOfMassInPlace(mass, p_QBcm_E);
  }

  /// Shifts `this` rotational inertia for a body (or composite body) B
  /// from about-point P to about-point Q via Bcm (B's center of mass).
  /// I.e., shifts `I_BP_E` to `I_BQ_E` (both are expressed-in frame E).
  /// @param mass The mass of body (or composite body) B.
  /// @param p_PBcm_E Position vector from P to Bcm, expressed-in frame E.
  /// @param p_QBcm_E Position vector from Q to Bcm, expressed-in frame E.
  /// @return A reference to `this` rotational inertia expressed-in frame E,
  ///         but with `this` shifted from about-point P to about-point Q,
  ///         i.e., returns I_BQ_E, B's rotational inertia about-point Q
  ///         expressed-in frame E.
  /// @throws std::exception for Debug builds if the rotational inertia that
  /// is shifted to about-point Q violates CouldBePhysicallyValid().
  /// @remark Negating either (or both) position vectors p_PBcm_E and p_QBcm_E
  ///         has no affect on the result.
  /// @remark This method is more efficient (by 6 multiplications) than first
  ///         shifting to the center of mass, then shifting away, e.g., as
  ///         (ShiftToCenterOfMassInPlace()).ShiftFromCenterOfMassInPlace();
  RotationalInertia<T>& ShiftToThenAwayFromCenterOfMassInPlace(
      const T& mass,
      const Vector3<T>& p_PBcm_E,
      const Vector3<T>& p_QBcm_E) {
    *this += mass * ShiftUnitMassBodyToThenAwayFromCenterOfMass(p_PBcm_E,
                                                                p_QBcm_E);
    return *this;
  }

  /// Calculates the rotational inertia that results from shifting `this`
  /// rotational inertia for a body (or composite body) B
  /// from about-point P to about-point Q via Bcm (B's center of mass).
  /// I.e., shifts `I_BP_E` to `I_BQ_E` (both are expressed-in frame E).
  /// @param mass The mass of body (or composite body) B.
  /// @param p_PBcm_E Position vector from P to Bcm, expressed-in frame E.
  /// @param p_QBcm_E Position vector from Q to Bcm, expressed-in frame E.
  /// @retval I_BQ_E, B's rotational inertia about-point Q expressed-in frame E.
  /// @throws std::exception for Debug builds if the rotational inertia that
  /// is shifted to about-point Q violates CouldBePhysicallyValid().
  /// @remark Negating either (or both) position vectors p_PBcm_E and p_QBcm_E
  ///         has no affect on the result.
  RotationalInertia<T> ShiftToThenAwayFromCenterOfMass(
      const T& mass,
      const Vector3<T>& p_PBcm_E,
      const Vector3<T>& p_QBcm_E) const __attribute__((warn_unused_result)) {
    return RotationalInertia(*this).ShiftToThenAwayFromCenterOfMassInPlace(
                                    mass, p_PBcm_E, p_QBcm_E);
  }
  ///@}

 protected:
  /// Subtracts a rotational inertia `I_BP_E` from `this` rotational inertia.
  /// No check is done to determine if the result is physically valid.
  /// @param I_BP_E Rotational inertia of a body (or composite body) B to
  ///        be subtracted from `this` rotational inertia.
  /// @return A reference to `this` rotational inertia. `this` changes
  ///         since rotational inertia `I_BP_E` has been subtracted from it.
  /// @see operator-().
  /// @warning This operator may produce an invalid rotational inertia.
  ///           Use operator-=() to perform necessary (but insufficient) checks
  ///           on the physical validity of the resulting rotational inertia.
  /// @note: Although this method is mathematically useful, it may result in a
  /// rotational inertia that is physically invalid.  This method helps perform
  /// intermediate calculations which do not necessarily represent a real
  /// rotational inertia.  For example, an efficient way to shift a rotational
  /// inertia from an arbitrary point P to an arbitrary point Q is mathematical
  /// equivalent to a + (b - c).  Although `a` must be physically valid and the
  /// result `a + (b - c)` must be physically valid, the intermediate quantity
  /// (b - c) is not necessarily physically valid.  This method allows (b - c)
  /// to be calculated without requiring (b - c) to be physically valid.
  /// @see operator-=().
  RotationalInertia<T>& MinusEqualsUnchecked(
      const RotationalInertia<T>& I_BP_E) {
    this->get_mutable_triangular_view() -= I_BP_E.get_matrix();
    return *this;
  }

 private:
  // Make RotationalInertia<Scalar> templated on any other type Scalar be a
  // friend of this class templated on T. That way the method
  // RotationalInertia<T>::cast<Scalar>() can make use of the private
  // constructor RotationalInertia<Scalar>(const Eigen::MatrixBase&) for an
  // Eigen expression templated on Scalar.
  template <typename> friend class RotationalInertia;

  // Constructs a rotational inertia for a particle Q whose position vector
  // from about-point P is p_PQ_E = xx̂ + yŷ + zẑ = [x, y, z]_E, where E is the
  // expressed-in frame.  Particle Q's mass (or unit mass) is included in the
  // first argument.  This constructor is private as it is a "helper" function.
  // In debug builds, throws std::exception if rotational inertia that is
  // constructed from these arguments violates CouldBePhysicallyValid().
  // @param p_PQ_E Position from about-point P to Q, expressed-in frame E.
  // @param mass_p_PQ_E The mass of particle Q multiplied by `p_PQ_E`.
  //                    If unit mass, this argument is simply p_PQ_E.
  // @retval I_QP_E, Q's rotational inertia about-point Q expressed-in frame E.
  RotationalInertia(const Vector3<T>& mass_p_PQ_E, const Vector3<T>& p_PQ_E) {
    const T& mx = mass_p_PQ_E(0);
    const T& my = mass_p_PQ_E(1);
    const T& mz = mass_p_PQ_E(2);
    const T& x = p_PQ_E(0);
    const T& y = p_PQ_E(1);
    const T& z = p_PQ_E(2);
    const T mxx = mx*x;
    const T myy = my*y;
    const T mzz = mz*z;
    set_moments_and_products_no_validity_check(myy + mzz, mxx + mzz, mxx + myy,
                                               -mx * y,   -mx * z,   -my * z);
    DRAKE_ASSERT_VOID(ThrowIfNotPhysicallyValid(__func__));
  }

  // Constructor from an Eigen expression that represents a matrix in ℝ³ˣ³ with
  // entries corresponding to inertia moments and products as described in this
  // class's documentation. This constructor will assert that I is a 3x3 matrix.
  // Runtime checks for physical validity can be disabled by setting the
  // optional argument `skip_validity_check` to `true`.
  // For internal use only.
  template <typename I_Type>
  explicit RotationalInertia(const Eigen::MatrixBase<I_Type>& I,
                             const bool skip_validity_check = false) {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<I_Type>, 3, 3);
    // Input matrix must be templated on the same scalar type as this inertia.
    static_assert(std::is_same_v<typename I_Type::Scalar, T>,
                  "Input argument I must be templated on the same scalar type "
                  "as this rotational inertia");
    I_SP_E_ = I;
    if (!skip_validity_check) {
      DRAKE_ASSERT_VOID(ThrowIfNotPhysicallyValid(__func__));
    }
  }

  // Sets this rotational inertia's moments and products of inertia. This method
  // intentionally avoids testing CouldBePhysicallyValid().  Some methods need
  // to be able to form non-physical rotational inertias (which are to be
  // subtracted or added to other rotational inertias to form valid ones).
  void set_moments_and_products_no_validity_check(
      const T& Ixx, const T& Iyy, const T& Izz,
      const T& Ixy, const T& Ixz, const T& Iyz) {
    // Note: The three upper off-diagonal matrix elements remain equal to NaN.
    static_assert(is_lower_triangular_order(1, 0), "Invalid indices");
    static_assert(is_lower_triangular_order(2, 0), "Invalid indices");
    static_assert(is_lower_triangular_order(2, 1), "Invalid indices");
    I_SP_E_(0, 0) = Ixx;  I_SP_E_(1, 1) = Iyy;  I_SP_E_(2, 2) = Izz;
    I_SP_E_(1, 0) = Ixy;  I_SP_E_(2, 0) = Ixz;  I_SP_E_(2, 1) = Iyz;
  }

  // Calculates the rotational inertia that must be added to account for
  // shifting the rotational inertia for a unit-mass body (or composite body) B
  // from about-point P to about-point Q via Bcm (B's center of mass).  In
  // other words, shifts `I_BP_E` to `I_BQ_E` (both are expressed-in frame E).
  // In debug builds, throws std::exception if rotational inertia that is
  // shifted to about-point Bcm or Q violates CouldBePhysicallyValid().
  // @param p_PBcm_E Position vector from P to Bcm, expressed-in frame E.
  // @param p_QBcm_E Position vector from Q to Bcm, expressed-in frame E.
  // @return A rotational inertia expressed-in frame E which when added to
  //         the rotational inertia I_BP_E produces I_BQ_E.  In other words,
  //         returns I_BQ_E - I_BP_E, expressed-in frame E.
  // @remark Negating either (or both) position vectors p_PBcm_E and p_QBcm_E
  //         has no affect on the result.
  RotationalInertia<T> ShiftUnitMassBodyToThenAwayFromCenterOfMass(
      const Vector3<T>& p_PBcm_E, const Vector3<T>& p_QBcm_E) {
    // Concept: Shift towards then away from the center of mass.
    // Math: Shift away from then towards the center of mass.
    RotationalInertia shift_away(p_QBcm_E, p_QBcm_E);
    RotationalInertia shift_towards(p_PBcm_E, p_PBcm_E);
    return shift_away.MinusEqualsUnchecked(shift_towards);
  }

  // This function returns true if arguments `i` and `j` access the lower-
  // triangular portion of the rotational matrix, otherwise false.
  static constexpr bool is_lower_triangular_order(int i, int j) {
    return i >= j;
  }

  // Utility method to swap matrix indexes (i, j) if i < j, which helps ensure
  // that only the lower-triangular part of the rotational inertia is used.
  static void check_and_swap(int* i, int* j) {
    if (!is_lower_triangular_order(*i, *j)) std::swap(*i, *j);
  }

  // Returns a constant reference to the underlying Eigen matrix. Notice that
  // since RotationalInertia only uses the lower-triangular portion of its
  // matrix, the three upper off-diagonal matrix elements will be NaN.
  // Most users won't call this method.
  const Matrix3<T>& get_matrix() const { return I_SP_E_; }

  // Returns a const Eigen view expression to the symmetric part of the matrix
  // in use by this RotationalInertia.
  const Eigen::SelfAdjointView<const Matrix3<T>, Eigen::Lower>
  get_symmetric_matrix_view() const {
    return I_SP_E_.template selfadjointView<Eigen::Lower>();
  }

  // Returns a mutable Eigen view expression to the symmetric part of the
  // matrix in use by RotationalInertia.
  // Note: operator=() is not defined for Eigen::SelfAdjointView and therefore
  // we need to return a TriangularView here.
  Eigen::TriangularView<Matrix3<T>, Eigen::Lower>
  get_mutable_triangular_view() {
    return I_SP_E_.template triangularView<Eigen::Lower>();
  }

  // Compares `this` rotational inertia to `other` rotational inertia within
  // `epsilon` (which specifies the value to which comparisons are performed).
  // `this` and `other` are considered approximately equal if:
  // ‖this - other‖∞  <  precision.
  // @param other Rotational inertia to compare with `this` rotational inertia.
  // @param   epsilon should be a real non-negative number, with units of
  //          inertia (e.g., kg*m^2).  It is usually small relative to the
  //          maximum moment of inertia in `this` or `other`.
  // @return `true` if the absolute value of each moment/product of inertia
  //          in `this` is within `epsilon` of the corresponding moment/
  //          product absolute value in `other`.  Otherwise returns `false`.
  // @note Trace() / 2 is a rotational inertia's maximum possible element,
  // e.g., consider: epsilon = 1E-9 * Trace()  (where 1E-9 is a heuristic).
  boolean<T> IsApproxMomentsAndProducts(const RotationalInertia& other,
                                        const T& epsilon) const {
    const Vector3<T> moment_difference = get_moments() - other.get_moments();
    const Vector3<T> product_difference = get_products() - other.get_products();
    const T moment_max = moment_difference.template lpNorm<Eigen::Infinity>();
    const T product_max = product_difference.template lpNorm<Eigen::Infinity>();
    return moment_max <= epsilon && product_max <= epsilon;
  }

  // Tests whether each moment of inertia is non-negative (to within `epsilon`)
  // and tests whether moments of inertia satisfy triangle-inequality.  The
  // triangle-inequality test requires `epsilon` when the sum of two moments are
  // nearly equal to the third one. Example: Ixx = Iyy = 50, Izz = 100.00000001,
  // or Ixx = -0.0001 (negative),  Ixx = 49.9999,  Iyy = 50.
  // A positive (non-zero) epsilon accounts for round-off errors, e.g., from
  // re-expressing inertia in another frame.
  // @param Ixx, Iyy, Izz moments of inertia for a generic rotational inertia,
  //        (i.e., not necessarily principal moments of inertia).
  // @param epsilon Real positive number that is much smaller than the largest
  //        possible element in a valid rotational inertia.  Heuristically,
  //       `epsilon` should probably be a small multiplier of Trace() / 2.
  // @note Denoting Imin and Imax as the smallest and largest possible moments
  //       of inertia in a valid rotational inertia, denoting Imed as the
  //       intermediate moment of inertia, and denoting tr as the trace of the
  //       rotational inertia (e.g., Ixx + Iyy + Izz), one can prove:
  //       0 <= Imin <= tr/3,   tr/3 <= Imed <= tr/2,   tr/3 <= Imax <= tr/2.
  //       If Imin == 0, then Imed == Imax == tr / 2.
  static boolean<T>
  AreMomentsOfInertiaNearPositiveAndSatisfyTriangleInequality(
      const T& Ixx, const T& Iyy, const T& Izz, const T& epsilon) {
    const auto are_moments_near_positive = AreMomentsOfInertiaNearPositive(
        Ixx, Iyy, Izz, epsilon);
    const auto is_triangle_inequality_satisified = Ixx + Iyy + epsilon >= Izz &&
                                                   Ixx + Iyy + epsilon >= Iyy &&
                                                   Iyy + Izz + epsilon >= Ixx;
    return are_moments_near_positive && is_triangle_inequality_satisified;
  }

  // Tests whether each moment of inertia is non-negative (to within epsilon).
  // This test allows for small (equal to -epsilon) negative moments of inertia
  // due to round-off errors, e.g., from expressing a rotational inertia.
  // @param Ixx, Iyy, Izz moments of inertia for a generic rotational inertia,
  //        (i.e., not necessarily principal moments of inertia).
  // @param epsilon Real positive number that is significantly smaller than the
  //        largest possible element in a valid rotational inertia.
  //        Heuristically, `epsilon` is a small multiplier of Trace() / 2.
  static boolean<T> AreMomentsOfInertiaNearPositive(
      const T& Ixx, const T& Iyy, const T& Izz, const T& epsilon) {
    return Ixx + epsilon >= 0  &&  Iyy + epsilon >= 0  &&  Izz + epsilon >= 0;
  }

  // ==========================================================================
  // The following set of methods, ThrowIfSomeCondition(), are used within
  // assertions or demands. We do not try to attempt a smart way throw based on
  // a given symbolic::Formula but instead we make these methods a no-throw
  // for non-numeric types.

  // This method is used to demand the physical validity of a RotationalInertia
  // at either construction or after an operation that could lead to
  // non-physical results when a user provides data that is not valid. For
  // numerical T-types this would imply computing the rotational inertia
  // eigenvalues and checking if they are positive and satisfy the triangle
  // inequality.
  template <typename T1 = T>
  typename std::enable_if_t<scalar_predicate<T1>::is_bool>
  ThrowIfNotPhysicallyValid(const char* func_name) {
    DRAKE_DEMAND(func_name != nullptr);
    if (!CouldBePhysicallyValid())
      ThrowNotPhysicallyValid(func_name);
  }


  // SFINAE for non-numeric types. See documentation in the implementation for
  // numeric types.
  template <typename T1 = T>
  typename std::enable_if_t<!scalar_predicate<T1>::is_bool>
  ThrowIfNotPhysicallyValid(const char*) {}

  [[noreturn]] void ThrowNotPhysicallyValid(const char* func_name) const;

  // Throws an exception if a rotational inertia is multiplied by a negative
  // number - which implies that the resulting rotational inertia is invalid.
  template <typename T1 = T>
  static typename std::enable_if_t<scalar_predicate<T1>::is_bool>
  ThrowIfMultiplyByNegativeScalar(const T& nonnegative_scalar) {
    if (nonnegative_scalar < 0) {
      throw std::logic_error("Error: Rotational inertia is multiplied by a "
                             "negative number.");
    }
  }

  // SFINAE for non-numeric types. See documentation in the implementation for
  // numeric types.
  template <typename T1 = T>
  static typename std::enable_if_t<!scalar_predicate<T1>::is_bool>
  ThrowIfMultiplyByNegativeScalar(const T&) {}

  // Throws an exception if a rotational inertia is divided by a non-positive
  // number - which implies that the resulting rotational inertia is invalid.
  template <typename T1 = T>
  static typename std::enable_if_t<scalar_predicate<T1>::is_bool>
  ThrowIfDivideByZeroOrNegativeScalar(const T& positive_scalar) {
    if (positive_scalar == 0)
      throw std::logic_error("Error: Rotational inertia is divided by 0.");
    if (positive_scalar < 0) {
      throw std::logic_error("Error: Rotational inertia is divided by a "
                             "negative number");
    }
  }

  // SFINAE for non-numeric types. See documentation in the implementation for
  // numeric types.
  template <typename T1 = T>
  static typename std::enable_if_t<!scalar_predicate<T1>::is_bool>
  ThrowIfDivideByZeroOrNegativeScalar(const T&) {}

  // The 3x3 inertia matrix is symmetric and its diagonal elements (moments of
  // inertia) and off-diagonal elements (products of inertia) are associated
  // with a body (or composite body) S, an about-point P, and an expressed-in
  // frame E.  However the user of this class is responsible for tracking S, P,
  // and E  (none of these are stored in this class).
  // The only data stored by the rotational inertia class is its inertia matrix.
  // Since the inertia matrix is symmetric, only the lower-triangular part of
  // the matrix is used.  All elements of the inertia matrix are initially set
  // to NaN which helps ensure the upper-triangular part is not used.
  Matrix3<T> I_SP_E_{Matrix3<T>::Constant(std::numeric_limits<
      typename Eigen::NumTraits<T>::Literal>::quiet_NaN())};
};

/// Writes an instance of RotationalInertia into a std::ostream.
/// @relates RotationalInertia
template <typename T>
std::ostream& operator<<(std::ostream& out, const RotationalInertia<T>& I);

// (This implementation is adapted from Simbody's Rotation::reexpressSymMat33.)
// Use sneaky tricks from Featherstone to rotate a symmetric dyadic matrix.
// See Featherstone 2008, Appendix A.5, pg 254.
//
// Let the current rotational inertia matrix be I=I_EE, meaning it is a dyadic
// expressed in E (only the expressed-in frame matters here so we're switching
// notation to emphasize that). We're given a rotation matrix R_AE and would
// like to efficiently calculate I' = I_AA = R_AE I_EE R_AEᵀ, writing the result
// in place.
//
// If the calculation were actually performed by multiplying three generic 3x3
// matrices, it would take 90 flops (as used here a "flop" is a floating-point
// multiply or add), or 75 flops if we leveraged the symmetry of the I_AA and
// I_EE inertia matrices (each matrix only has 6 unique elements). However,
// with the clever algorithm below, the calculation cost is only 57 flops.
//
// We'll work with I's elements: I =[ a d e ]
//                                  [ d b f ]
//                                  [ e f c ]
//
// First, split I into I=L+D+v× with v=[-f e 0]ᵀ (× means cross product matrix):
//        [a-c   d   0]       [c 0 0]        [ 0  0  e]
//    L = [ d   b-c  0]   D = [0 c 0]   v× = [ 0  0  f]
//        [2e   2f   0]       [0 0 c]        [-e -f  0]
// (4 flops to calculate L)
//
// A cross product matrix identity says R v× Rᵀ=(Rv)×, so:
//    I' = R I Rᵀ = R L Rᵀ + D + (Rv)×.
// Let Y' = R L, Z = Y' Rᵀ. We only need the lower triangle of Z and a
// 2x2 square of Y'.
//
// Don't-care's below are marked "-". Below we'll use square bracket [i]
// index of a matrix to mean "row i", round bracket (j) means "column j".
//
//        [  -   -  0 ]
//   Y' = [ Y00 Y01 0 ]   Y = [ R[1]•L(0)  R[1]•L(1) ]     20 flops
//        [ Y10 Y11 0 ]       [ R[2]•L(0)  R[2]•L(1) ]
//
//   Z = [   Z00          -          -     ]
//       [ Y[0]•R[0]  Y[0]•R[1]      -     ]   15 flops (use only 2
//       [ Y[1]•R[0]  Y[1]•R[1]  Y[1]•R[2] ]   elements of R's rows)
//
//   Z00 = (L00+L11)-(Z11+Z22)  3 flops (because rotation preserves trace)
//
//         [e R01 - f R00]           [  0       -    -  ]
//    Rv = [e R11 - f R10]   (Rv)× = [ Rv[2]    0    -  ]
//         [e R21 - f R20]           [-Rv[1]  Rv[0]  0  ]
// (Rv is 9 flops)
//
//        [ Z00 + c          -           -    ]
//   I' = [ Z10 + Rv[2]  Z11 + c         -    ]
//        [ Z20 - Rv[1]  Z21 + Rv[0]  Z22 + c ]
//
// which takes 6 more flops. Total 6 + 9Rv + 18Z + 20Y + 4L = 57.
//
// (Looking at the generated assembly code for gcc 7.5 -O2 in Drake showed
// exactly 57 flops. clang 9.0 -O2 generated a few extra flops but in many fewer
// instructions since it was able to make much better use of SSE2 packed
// instructions. Both compilers were fully able to inline the Eigen methods here
// so there are no loops or function calls.)
template <typename T>
RotationalInertia<T>& RotationalInertia<T>::ReExpressInPlace(
    const math::RotationMatrix<T>& R_AE) {
  const Matrix3<T>& R = R_AE.matrix();

  // We're going to write back into this lower triangle.
  T& a = I_SP_E_(0, 0);
  T& d = I_SP_E_(1, 0); T& b = I_SP_E_(1, 1);
  T& e = I_SP_E_(2, 0); T& f = I_SP_E_(2, 1); T& c = I_SP_E_(2, 2);

  // Avoid use of Eigen's comma initializer since the compilers don't
  // reliably inline it.
  Eigen::Matrix<T, 3, 2> L;
  L(0, 0) = a-c; L(0, 1) = d;
  L(1, 0) = d;   L(1, 1) = b-c;
  L(2, 0) = 2*e; L(2, 1) = 2*f;

  // For convenience below, the first two rows of Rᵀ.
  Eigen::Matrix<T, 2, 3> Rt;
  Rt(0, 0) = R(0, 0); Rt(0, 1) = R(1, 0); Rt(0, 2) = R(2, 0);
  Rt(1, 0) = R(0, 1); Rt(1, 1) = R(1, 1); Rt(1, 2) = R(2, 1);

  const Vector3<T> Rv(e * Rt.row(1) - f * Rt.row(0));

  Matrix2<T> Y;
  Y(0, 0) = R.row(1).dot(L.col(0)); Y(0, 1) = R.row(1).dot(L.col(1));
  Y(1, 0) = R.row(2).dot(L.col(0)); Y(1, 1) = R.row(2).dot(L.col(1));

  // We'll do Z elementwise due to element interdependence.
  const T Z11 = Y.row(0) * Rt.col(1);
  const T Z22 = Y.row(1) * Rt.col(2);
  const T Z00 = (L(0, 0) + L(1, 1)) - (Z11 + Z22);
  const T Z10 = Y.row(0) * Rt.col(0);
  const T Z20 = Y.row(1) * Rt.col(0);
  const T Z21 = Y.row(1) * Rt.col(1);

  // Assign result back into lower triangle. Don't set c until we're done
  // with the previous value!
  a = Z00 + c;
  d = Z10 + Rv[2]; b = Z11 + c;
  e = Z20 - Rv[1]; f = Z21 + Rv[0]; c += Z22;

  // If both `this` and `R_AE` were valid upon entry to this method, the
  // returned rotational inertia should be valid.  Otherwise, it may not be.
  DRAKE_ASSERT_VOID(ThrowIfNotPhysicallyValid(__func__));
  return *this;
}

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::RotationalInertia)
