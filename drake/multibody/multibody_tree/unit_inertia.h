#pragma once

#include <memory>
#include <sstream>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"

namespace drake {
namespace multibody {

template <typename T>
class UnitInertia : public RotationalInertia<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(UnitInertia)

  /// Default RotationalInertia constructor. All entries are set to NaN for a
  /// quick detection of un-initialized values.
  UnitInertia() {}

  /// Creates a principal unit inertia with identical diagonal elements
  /// equal to @p I and zero products of inertia.
  /// As examples, consider the moments of inertia taken about their geometric
  /// center for a sphere or a cube.
  /// Throws an exception if `I` is negative.
  /// @see UnitInertia::SolidSphere() and UnitInertia::SolidCube().
  UnitInertia(const T& I) : RotationalInertia<T>(I) {}

  /// Creates a principal axes rotational inertia matrix for which the products
  /// of inertia are zero and the moments of inertia are given by `Ixx`, `Iyy`
  /// and `Izz`.
  /// Throws an exception if any of the provided moments is negative or the
  /// resulting inertia is not physically valid according to
  /// RotationalInertia::CouldBePhysicallyValid().
  UnitInertia(const T& Ixx, const T& Iyy, const T& Izz) : 
      RotationalInertia<T>(Ixx, Iyy, Izz) {}

  /// Creates a general unit inertia matrix with non-zero off-diagonal
  /// elements where the six components of the rotational intertia on a given
  /// frame `E` need to be provided.
  /// Throws an exception if the resulting inertia is invalid according to
  /// RotationalInertia::CouldBePhysicallyValid().
  UnitInertia(const T& Ixx, const T& Iyy, const T& Izz,
              const T& Ixy, const T& Ixz, const T& Iyz) :
      RotationalInertia<T>(Ixx, Iyy, Izz, Ixy, Ixz, Iyz) {}

  /// Constructs a UnitInertia from a RotationalInertia. This constructor has
  /// no way to verify that the input rotational inertia IS a unit inertia.
  /// It is the responsability of the user to pass a valid unit inertia.
  UnitInertia(const RotationalInertia<T>& I) : RotationalInertia<T>(I) {}
  
  /// Given `this` rotational inertia `G_Bo_F` about `Bo` and expressed in frame
  /// `F`, this method computes the same unit inertia re-expressed in another
  /// frame `A` as `I_Bo_A = R_AF * G_Bo_F * (R_AF)ᵀ`.
  /// This operation is performed in-place modifying the original object.
  /// @param[in] R_AF Rotation matrix from frame `F` to frame `A`.
  /// @returns A reference to `this` unit inertia about `Bo` but now
  ///          re-expressed in frame `A`.
  UnitInertia<T>& ReExpressInPlace(const Matrix3<T>& R_AF) {
    return RotationalInertia<T>::ReExpressInPlace(R_AF);
  }

  /// Given `this` rotational inertia `G_Bo_F` about `Bo` and expressed in frame
  /// `F`, this method computes the same unit inertia re-expressed in another
  /// frame `A` as `I_Bo_A = R_AF * G_Bo_F * (R_AF)ᵀ`.
  /// @param[in] R_AF Rotation matrix from frame `F` to frame `A`.
  /// @retval G_Bo_A The same rotational inertia about `Bo` but now
  ///                re-expressed in frame`A`.
  UnitInertia<T> ReExpress(const Matrix3<T>& R_AF) const {
    return UnitInertia<T>(RotationalInertia<T>::ReExpress(R_AF));
  }

  /// For a central unit inertia `G_Bcm_E` computed about a body's center of
  /// mass `Bcm` and expressed in a frame `E`, this method shifts this inertia
  /// using the parallel axis theorem to be computed about a point `Qo`.
  /// This operation is performed in-place modifying the original object.
  /// @param[in] p_BcmQo_E A vector from the body's center of mass `Bcm` to
  ///                      point `Qo` expressed in the same frame `E` in which
  ///                      `this` inertia is expressed.
  /// @returns A reference to `this` unit inertia but now computed about
  ///          point `Qo` and expressed in frame `E`.
  UnitInertia<T>& ShiftFromCenterOfMassInPlace(const Vector3<T>& p_BcmQo_E) {
    RotationalInertia<T>::operator+=(PointMass(p_BcmQo_E));
    return *this;
  }

  /// For a central unit inertia `G_Bcm_E` computed about a body's center of
  /// mass `Bcm` and expressed in a frame `E`, this method shifts this inertia
  /// using the parallel axis theorem to be computed about a point `Qo`.
  /// @param[in] p_BcmQo_E A vector from the body's center of mass `Bcm` to
  ///                      point `Qo` expressed in the same frame `E` in which
  ///                      `this` inertia is expressed.
  /// @retval G_Qo_E This same unit inertia computed about a point `Qo` and
  ///                expressed in frame `E`.
  UnitInertia<T> ShiftFromCenterOfMass(const Vector3<T>& p_BcmQo_E) const {
    return UnitInertia<T>(*this).ShiftFromCenterOfMassInPlace(p_BcmQo_E);
  }

  /// @name UnitInertia's for common 3D objects.
  /// The following methods allow to construct UnitInertia's for common 3D
  /// objects such as boxes, spheres, rods and others.
  /// The UnitInertias computed correspond to objects with unit mass and most
  /// of the times are computed about these objects centers of mass and in a
  /// frame aligned with their principal axes.
  /// To construct general RotationalInertia's use these methods along with
  /// ShiftFromCenterOfMassInPlace() to move the point about which the inertia
  /// is computed and use ReExpress() to express in a different frame.
  //@{

  /// Construct a unit inertia for a point of unit mass located at point `p_F`
  /// measured and expressed in frame `F`.
  /// The unit inertia `G_Fo_F` about the origin `Fo` of frame `F` and expressed
  /// in `F` for this unit mass point equals to the square of the cross product
  /// matrix of `p_F`: <pre>
  ///   G_Fo_F = [p_F]^2 = [p_F]ᵀ * [p_F] = -[p_F] * [p_F]
  /// </pre>
  /// where `[v]` denotes the cross product matrix of a vector `v` such that the
  /// cross product with another vector `a` can be obtained as
  /// `v.cross(a) = [v] * a`. The cross product matrix `[v]` is skew-symmetric.
  /// The square of the cross product matrix is a symmetric matrix with
  /// non-negative diagonals and obeys the triangle inequality.
  /// Matrix `S(v)` can be used to compute the triple vector product
  /// `v x (v x a) = v.cross(v.cross(a)) = S(v) * a`.
  static UnitInertia<T> PointMass(const Vector3<T>& p_F) {
    const Vector3<T> p2m = p_F.cwiseAbs2();
    const T mp0 = -p_F(0);
    const T mp1 = -p_F(1);
    return UnitInertia<T>(
        /*          Ixx,             Iyy,             Izz */
        p2m[1] + p2m[2], p2m[0] + p2m[2], p2m[0] + p2m[1],
        /*       Ixy,          Ixz,          Iyz */
        mp0 * p_F[1], mp0 * p_F[2], mp1 * p_F[2]);
  }

  /// Computes the unit inertia for a unit-mass solid sphere of radius
  /// @p r taken about its center.
  static UnitInertia<T> SolidSphere(const T& r) {
    return UnitInertia(T(0.4) * r * r);
  }

  /// Computes the unit inertia for a unit-mass hollow sphere consisting
  /// of an infinitesimally thin shell of radius @p r. The unit inertia is
  /// taken about the center of the sphere.
  static UnitInertia<T> HollowSphere(const T& r) {
    return UnitInertia(T(2)/T(3) * r * r);
  }

  /// Computes the unit inertia for a unit-mass solid box taken about its
  /// geometric center. If one length is zero the inertia corresponds to that of
  /// a thin rectangular sheet. If two lengths are zero the inertia corresponds
  /// to that of a thin rod in the remaining direction.
  /// @param[in] Lx The length of the box edge in the principal x-axis.
  /// @param[in] Ly The length of the box edge in the principal y-axis.
  /// @param[in] Lz The length of the box edge in the principal z-axis.
  static UnitInertia<T> SolidBox(const T& Lx, const T& Ly, const T& Lz) {
    const T one_twelfth = T(1) / T(12);
    const T Lx2 = Lx * Lx, Ly2 = Ly * Ly, Lz2 = Lz * Lz;
    return UnitInertia(
        one_twelfth * (Ly2 + Lz2),
        one_twelfth * (Lx2 + Lz2),
        one_twelfth * (Lx2 + Ly2));
  }

  /// Computes the unit inertia for a unit-mass solid cube (a box with
  /// equal sized sides) taken about its geometric center.
  /// @param[in] L The length of each of the cube's sides.
  static UnitInertia<T> SolidCube(const T& L) {
    return SolidBox(L, L, L);
  }

  /// Computes the unit inertia for a unit-mass cylinder oriented along the
  /// z-axis computed about its center.
  /// @param[in] r The radius of the cylinder.
  /// @param[in] L The length of the cylinder.
  static UnitInertia<T> SolidCylinder(const T& r, const T& L) {
    const T Iz = r * r / T(2);
    const T Ix = (T(3) * r * r + L * L) / T(12);
    return UnitInertia(Ix, Ix, Iz);
  }

  /// Computes the unit inertia for a unit-mass cylinder oriented along the
  /// z-axis computed about a point at the center of its base.
  /// @param[in] r The radius of the cylinder.
  /// @param[in] L The length of the cylinder.
  static UnitInertia<T> SolidCylinderAboutEnd(const T& r, const T& L) {
    const T Iz = r * r / T(2);
    const T Ix = (T(3) * r * r + L * L) / T(12) + L * L / T(4);
    return UnitInertia(Ix, Ix, Iz);
  }
  // End of Doxygen group
  //@}

 private:
  // Disable here operations that while well defined for the general
  // RotationalInertia class, would otherwise result in general in non-unit
  // inertias.
  RotationalInertia<T>& operator+=(const RotationalInertia<T>& I_Bo_F) {}

  // Disable operations to any scalar that has an implicit conversion to int
  // defined.
  void operator+=(int) {}
  void operator-=(int) {}
  void operator*=(int) {}
  void operator/=(int) {}
};

}  // namespace multibody
}  // namespace drake
