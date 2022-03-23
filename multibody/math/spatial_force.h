#pragma once

#ifndef DRAKE_SPATIAL_ALGEBRA_HEADER
#error Please include "drake/multibody/math/spatial_algebra.h", not this file.
#endif

#include <limits>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/math/spatial_vector.h"

namespace drake {
namespace multibody {

// Forward declaration to define dot product with a spatial velocity.
template <typename T> class SpatialVelocity;

/// This class represents a _spatial force_ (also called a _wrench_) and has 6
/// elements with a torque 𝛕 (3-element vector) on top of a force 𝐟 (3-element
/// vector). Frequently, a spatial force represents the replacement of a set S
/// of forces on a frame B with an equivalent set consisting of a torque 𝛕
/// applied to frame B which is equal to the moment of the set S about a point
/// Bp of B together with a force 𝐟 applied to Bp, where 𝐟 is equal to set S's
/// resultant force.  This class assumes that both the torque 𝛕 and force 𝐟 have
/// the same _expressed-in_ frame E. This class only stores 6 elements (namely
/// 𝛕 and 𝐟) and does not store the underlying frame B, expressed-in frame E, or
/// application point Bp. The user is responsible for explicitly tracking these
/// underlying quantities with @ref multibody_quantities "monogram notation".
/// For example, F_B_E denotes a spatial force on frame B, expressed in frame E
/// and contains tau_B_E (torque applied to frame B, expressed in frame E) and
/// f_Bo_E (force applied frame B's origin point Bo, expressed in E).
///
/// The monogram notation F_Bp has a typeset equivalent @f${F^{Bp}}@f$ which
/// denotes the spatial force applied to point Bp of frame B.  F_Bp contains a
/// torque tau_B (@f${\tau^B}@f$) applied to frame B and a force f_Bp
/// (@f${f^{Bp}}@f$) applied to point Bp of frame B.
/// Details on spatial vectors and monogram notation are in sections
/// @ref multibody_spatial_vectors and @ref multibody_quantities.
///
/// @tparam_default_scalar
template <typename T>
class SpatialForce : public SpatialVector<SpatialForce, T> {
  // We need the fully qualified class name below for the clang compiler to
  // work. Without qualifiers the code is legal according to the C++11 standard
  // but the clang compiler still gets confused. See:
  // http://stackoverflow.com/questions/17687459/clang-not-accepting-use-of-template-template-parameter-when-using-crtp
  typedef SpatialVector<::drake::multibody::SpatialForce, T> Base;

 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpatialForce)

  /// Default constructor. In Release builds, all 6 elements of a newly
  /// constructed spatial force are uninitialized (for speed). In Debug
  /// builds, the 6 elements are set to NaN so that invalid operations on an
  /// uninitialized spatial force fail fast (fast bug detection).
  SpatialForce() : Base() {}

  /// Constructs a spatial force from a torque 𝛕 (tau) and a force 𝐟.
  SpatialForce(const Eigen::Ref<const Vector3<T>>& tau,
               const Eigen::Ref<const Vector3<T>>& f) : Base(tau, f) {}

  /// Constructs a spatial force from an Eigen expression that represents a
  /// 6-element vector, i.e., a 3-element torque 𝛕 and a 3-element force 𝐟.
  /// This constructor will assert the size of `A` is six (6) either at
  /// compile-time for fixed sized Eigen expressions or at run-time for
  /// dynamic sized Eigen expressions.
  template <typename Derived>
  explicit SpatialForce(const Eigen::MatrixBase<Derived>& F) : Base(F) {}

  /// In-place shift of a %SpatialForce from one point of frame B to another
  /// point of frame B.  On entry, `this` is F_Bp_E (spatial force on point Bp
  /// of frame B, expressed in a frame E). On return `this` is modified to
  /// F_Bq_E (spatial force on point Bq of frame B, expressed in a frame E).
  /// @param[in] offset which is the position vector p_BpBq_E from point Bp
  /// (fixed on frame B) to point Bq (fixed on frame B), expressed in frame E.
  /// p_BpBq_E must have the same expressed-in frame E as `this` spatial force.
  /// @retval F_Bq_E reference to `this` spatial velocity which has been
  /// modified to be the spatial force on Bq, expressed in frame E.
  /// The components of F_Bq_E are calculated as: <pre>
  ///    τ_B  = τ_B -  p_BpBq x f_Bp    (the torque 𝛕 stored in `this` changes).
  ///  f_Bq_E = f_Bp_E              (the force 𝐟 stored in `this` is unchanged).
  /// </pre>
  /// Note: Spatial force shift similar to SpatialMomentum and in a transpose
  /// way to SpatialVelocity.
  /// @see the alternative ShiftInPlace() function and its related Shift()
  /// function to shift multiple spatial forces. Use the other Shift() function
  /// to shift only one spatial force without modifying `this`.
  SpatialForce<T>& ShiftInPlace(const Vector3<T>& p_BpBq_E) {
    this->rotational() -= p_BpBq_E.cross(this->translational());
    return *this;
    // Note: this operation is linear. [Jain 2010], (§1.5, page 15) uses the
    // "rigid body transformation operator" to write this as:
    //   F_Bq = Φ(p_BqBp)F_Bp = Φ(-p_BpBq)F_Bp
    //   where Φ(p_BpBq) is the linear operator:
    //   Φ(p_BpBq) = | I₃  p_BpBqx |
    //               | 0        I₃ |
    // where `p_BpBqx` denotes the cross product skew-symmetric matrix such that
    // `p_BpBqx vec = p_BpBq x vec` (where vec is any vector).
    // This same operator shifts spatial momentum in an analogous way (see
    // SpatialMomentum::Shift()) whereas the transpose of this operator shifts
    // spatial velocity (see SpatialVelocity::Shift()).
    //
    // - [Jain 2010] Jain, A., 2010. Robot and multibody dynamics: analysis and
    //               algorithms. Springer Science & Business Media, pp. 123-130.
  }

  /// Shifts a matrix of spatial forces from one point of frame B to another
  /// point of frame B.
  /// @param[in,out] F_Bp_E_all 6 x n matrix where each of the n columns is
  /// regarded as a spatial force. On input, each spatial force is applied to a
  /// point Bp of frame B.  On output, each spatial force has been shifted to
  /// a point Bq of frame B (i.e., F_Bq_E_All on output).
  /// @param[in] p_BpBq_E is the position vector p_BpBq_E from point Bp
  /// (fixed on frame B) to point Bq (fixed on frame B), expressed in frame E.
  /// p_BpBq_E must have the same expressed-in frame E as the spatial forces
  /// stored in F_Bp_E_All (spatial forces on Bp, expressed in frame E).
  /// @see the alternative ShiftInPlace() function to shift only one spatial
  /// force and for details on how F_Bq_E is calculated.  See the related
  /// Shift() functions to shift one spatial force or multiple spatial forces.
  static void ShiftInPlace(EigenPtr<Matrix6X<T>> F_Bp_E_all,
                           const Vector3<T>& p_BpBq_E) {
    DRAKE_ASSERT(F_Bp_E_all != nullptr);  // ASSERT because inner loop method.
    const int ncol = F_Bp_E_all->cols();
    for (int j = 0; j < ncol; ++j) {
      // These are Eigen intermediate types; better not to look!
      auto F_Bp_E = F_Bp_E_all->col(j);
      auto torque = F_Bp_E.template head<3>();
      const auto force = F_Bp_E.template tail<3>();
      torque -= p_BpBq_E.cross(force);
    }
    // F_Bp_E_all should now be called F_Bq_E_all.
  }

  /// Shifts a %SpatialForce from one point of frame B to another point of
  /// frame B.
  /// @param[in] offset which is the position vector p_BpBq_E from point Bp
  /// (fixed on frame B) to point Bq (fixed on frame B), expressed in frame E.
  /// p_BpBq_E must have the same expressed-in frame E as `this` spatial force,
  /// where `this` is F_Bp_E (spatial force on Bp, expressed in frame E).
  /// @retval F_Bq_E which is the spatial force on Bq, expressed in frame E.
  /// @see the alternative Shift() function and its related ShiftInPlace()
  /// function to shift multiple spatial forces. There is another ShiftInPlace()
  /// function that shifts only one spatial force and which modifies `this`.
  SpatialForce<T> Shift(const Vector3<T>& p_BpBq_E) const {
    return SpatialForce<T>(*this).ShiftInPlace(p_BpBq_E);
  }

  /// Shifts a matrix of spatial forces from one point of frame B to another
  /// point of frame B.
  /// @param[in] F_Bp_E_all 6 x n matrix where each of the n columns is regarded
  /// as a spatial force applied to a point Bp of frame B and where each
  /// spatial force has an expressed-in frame E.
  /// @param[in] p_BpBq_E is the position vector p_BpBq_E from point Bp
  /// (fixed on frame B) to point Bq (fixed on frame B), expressed in frame E.
  /// p_BpBq_E must have the same expressed-in frame E as that in F_Bp_E_All.
  /// @param[out] F_Bq_E_all 6 x n matrix where each of the n column is regarded
  /// as a spatial force. On output, each contains a spatial force which was
  /// shifted from point Bp (fixed on frame B) to point Bq (fixed on frame B).
  /// Each of spatial forces contained in F_Bq_E_all are expressed in frame E.
  /// @see the alternative Shift() function and its related ShiftInPlace()
  /// function to shift only one spatial force. There is another ShiftInPlace()
  /// function that shifts multiple spatial forces.
  /// @pre F_Bq_E_all must be non-null and must point to a 6 x n matrix (i.e.,
  /// it must be the same size as the input matrix F_Bp_E_all).
  /// @note Although this Shift() function will work properly if the input and
  /// output matrices are the same (i.e., F_Bp_E_all = F_Bq_E_all), it is faster
  /// and more efficient if these are the same to avoid needless copying.
  static void Shift(const Eigen::Ref<const Matrix6X<T>>& F_Bp_E_all,
                    const Vector3<T>& p_BpBq_E,
                    EigenPtr<Matrix6X<T>> F_Bq_E_all) {
    DRAKE_DEMAND(F_Bq_E_all != nullptr);
    DRAKE_DEMAND(F_Bq_E_all->cols() == F_Bp_E_all.cols());
    *F_Bq_E_all = F_Bp_E_all;
    ShiftInPlace(F_Bq_E_all, p_BpBq_E);
  }

  /// Calculates the power generated by a spatial force.
  /// For an arbitrary frame B, calculates the dot-product of `this` = F_B_E
  /// (frame B's spatial force, expressed in frame E) with V_MB_E (frame B's
  /// spatial velocity measured in a frame M, expressed in a frame E).
  /// @param[in] velocity which is V_MB_E, frame B's spatial velocity measured
  /// in frame M, expressed in the same frame E as `this` = F_B_E.
  /// @returns Power of spatial force F_B_E in frame M, i.e., F_B_E ⋅ V_MB_E.
  /// @note Just as equating force 𝐅 to mass * acceleration as 𝐅 = m𝐚 relies
  /// on acceleration 𝐚 being measured in a world frame (also called an inertial
  /// or Newtonian frame), equating power = dK/dt (where K is kinetic energy)
  /// relies on K being measured in a world frame.  Hence, it is unusual to use
  /// this method unless frame M is the world frame W.
  /// @note Although the spatial vectors F_B_E and V_MB_E must have the same
  /// expressed-in frame E, the returned scalar is independent of frame E.
  inline T dot(const SpatialVelocity<T>& velocity) const;
  // The dot() method is implemented in spatial_velocity.h. We need the inline
  // keyword to ensure the method is still inlined even with `extern template`.
};

/// Adds two spatial forces by simply adding their 6 underlying elements.
/// @param[in] F1_E spatial force expressed in the same frame E as F2_E.
/// @param[in] F2_E spatial force expressed in the same frame E as F1_E.
/// @note The general utility of this operator+() function seems limited to
/// situations when F1 and F2 are associated with different sets of forces,
/// but are applied to the same frame B, with same application point Bp, and
/// have the same expressed-in frame E.
/// @relates SpatialForce
template <typename T>
inline SpatialForce<T> operator+(const SpatialForce<T>& F1_E,
                                 const SpatialForce<T>& F2_E) {
  // Although this operator+() function simply calls an associated
  // SpatialVector operator+=() function, it is needed for documentation.
  return SpatialForce<T>(F1_E) += F2_E;
}

/// Subtracts spatial forces by simply subtracting their 6 underlying elements.
/// @param[in] F1_E spatial force expressed in the same frame E as F2_E.
/// @param[in] F2_E spatial force expressed in the same frame E as F1_E.
/// @note The general utility of this operator-() function seems limited to
/// situations when F1 and F2 are associated with different sets of forces,
/// but are applied to the same frame B, with same application point Bp, and
/// have the same expressed-in frame E.
/// @relates SpatialForce
template <typename T>
inline SpatialForce<T> operator-(const SpatialForce<T>& F1_E,
                                 const SpatialForce<T>& F2_E) {
  // Although this operator-() function simply calls an associated
  // SpatialVector operator-=() function, it is needed for documentation.
  return SpatialForce<T>(F1_E) -= F2_E;
}

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::SpatialForce)
