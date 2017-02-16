#pragma once

#include <memory>
#include <sstream>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"

namespace drake {
namespace multibody {

template <typename T>
class UnitInertia : public RotationalInertia<T> {
 public:
  /// Default UnitInertia constructor. Everything is left initialiezed to
  /// NaN for a quick detection of un-initialized values.
  UnitInertia() {}

  // Default copy constructor and copy assignment.
  UnitInertia(const UnitInertia<T>& other) = default;
  UnitInertia& operator=(const UnitInertia<T>& other) = default;

  /// Creates a principal unit inertia with identical diagonal elements
  /// equal to @p I and zero products of inertia.
  /// As examples, consider the moments of inertia taken about their geometric
  /// center for a sphere or a cube.
  /// @see UnitInertia::SolidSphere() and UnitInertia::SolidCube().
  UnitInertia(const T& I) : RotationalInertia<T>(I) {}

  /// Create a principal axes unit inertia matrix for wich off-diagonal
  /// elements are zero.
  UnitInertia(const T& Ixx, const T& Iyy, const T& Izz) : 
      RotationalInertia<T>(Ixx, Iyy, Izz) {}

  /// Creates a general unit inertia matrix with non-zero off-diagonal
  /// elements.
  UnitInertia(const T& Ixx, const T& Iyy, const T& Izz,
              const T& Ixy, const T& Ixz, const T& Iyz) :
      RotationalInertia<T>(Ixx, Iyy, Izz, Ixy, Ixz, Iyz) {}

  /// Constructs a UnitInertia from an Eigen matrix expression.
  template<typename Derived>
  UnitInertia(const Eigen::MatrixBase<Derived>& m) : 
      RotationalInertia<T>(m) {}

  /// Assignment operator from a general Eigen expression.
  // This method allows you to assign Eigen expressions to a UnitInertia.
  template<typename Derived>
  UnitInertia& operator=(const Eigen::MatrixBase<Derived>& EigenMatrix)
  {
    RotationalInertia<T>::operator=(EigenMatrix);
    return *this;
  }

  /// Constructs a UnitInertia from a RotationalInertia. This constructor has
  /// no way to verify that the input rotational inertia IS a unit inertia.
  /// It is the responsability of the user to pass a valid unit inertia.
  UnitInertia(const RotationalInertia<T>& I) : RotationalInertia<T>(I) {}
  
  /// Given this unit inertia `G_Bo_F` about `Bo` and expressed in frame
  /// `F`, this method computes the same inertia re-expressed in another
  /// frame `A`.
  /// This operation is performed in-place modifying the original object.
  /// @param[in] R_AF Rotation matrix from frame `F` to frame `A`.
  /// @returns A reference to `this` unit inertia about `Bo` but now
  /// re-expressed in frame `A`.
  UnitInertia& ReExpressInPlace(const Matrix3<T>& R_AF) {
    return RotationalInertia<T>::ReExpressInPlace(R_AF);
  }

  /// Given this unit inertia `G_Bo_F` about `Bo` and expressed in
  /// frame `F`, this method computes the same inertia re-expressed in another
  /// frame `A`.
  /// @param[in] R_AF Rotation matrix from frame `F` to frame `A`.
  /// @returns G_Bo_A The same unit inertia bout `Bo` expressed in frame
  /// `A`.
  UnitInertia ReExpress(const Matrix3<T>& R_AF) const {
    return UnitInertia<T>(RotationalInertia<T>::ReExpress(R_AF));
  }

  /// @name UnitInertia's for common 3D objects.
  /// The following methods allow to construct UnitInertia's for common 3D
  /// objects such as boxes, spheres, rods and others.
  /// The UnitInertias computed correspond to objects with unit mass and most
  /// of the times are computed about these objects centers of mass and in a
  /// frame aligned with their principal axes.
  /// To construct general RotationalInertia's use these methods along with
  /// Shift() to move the point about which the inertia is computed and use
  /// ReExpress() to express in a different frame.
  //@{

  /// Computes the unit inertia for a unit-mass solid sphere of radius
  /// @p r taken about its center.
  static UnitInertia SolidSphere(const T& r) {
    return UnitInertia(T(0.4) * r * r);
  }

  /// Computes the unit inertia for a unit-mass hollow sphere consisting
  /// of an infinitesimally thin shell of radius @p r. The unit inertia is
  /// taken about the center of the sphere.
  static UnitInertia HollowSphere(const T& r) {
    return UnitInertia(T(2)/T(3) * r * r);
  }

  /// Computes the unit inertia for a unit-mass solid box taken about its
  /// geometric center. If one length is zero the inertia corresponds to that of
  /// a thin rectangular sheet. If two lengths are zero the inertia corresponds
  /// to that of a thin rod in the remaining direction.
  /// @param[in] Lx The length of the box edge in the principal x-axis.
  /// @param[in] Ly The length of the box edge in the principal y-axis.
  /// @param[in] Lz The length of the box edge in the principal z-axis.
  static UnitInertia SolidBox(const T& Lx, const T& Ly, const T& Lz) {
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
  static UnitInertia SolidCube(const T& L) {
    return SolidBox(L, L, L);
  }

  /// Computes the unit inertia for a unit-mass rod along the z-axis
  /// rotationg about its center.
  /// @param[in] r The radius of the rod.
  /// @param[in] L The length of the rod.
  static UnitInertia SolidRod(const T& r, const T& L) {
    const T Iz = r * r / T(2);
    const T Ix = L * L / T(12);
    return UnitInertia(Ix, Ix, Iz);
  }

  /// Computes the unit inertia for a unit-mass rod along the z-axis
  /// rotationg about one end.
  /// @param[in] r The radius of the rod.
  /// @param[in] L The length of the rod.
  static UnitInertia SolidRodAboutEnd(const T& r, const T& L) {
    const T Iz = r * r / T(2);
    const T Ix = L * L / T(3);
    return UnitInertia(Ix, Ix, Iz);
  }
  // End of Doxygen group "
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
