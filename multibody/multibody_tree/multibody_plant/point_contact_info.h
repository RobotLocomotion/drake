#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace multibody_plant {

/**
 The data for a single contact force applied to one body of a contacting pair.

 Ultimately, a contact force consists of an application point, a spatial
 force, and a unit vector. The spatial force includes:
    - a translational force, that is a pure force applied at a point,
    - a torque, the rotational force. This is not the same as the moment
      induced by the translational force.  This torque can arise from a
      combination of different sources (e.g., torsional friction, the sum of
      various moments, etc.)

 The unit normal indicates the normal direction of the translational force.
 Used to decompose the force into normal and tangential components. The
 normal is typically defined by the contact normal.

 A PointContactInfo makes no assumptions about the frame in which it is defined
 except that the vectors representing application point location, contact
 normal, force, and torque are all expressed in a common frame, with the
 application point measured from the origin of that frame. Every external
 instantiation of PointContactInfo must make it clear which frame is being used.

 @tparam T The scalar type. Must be a valid Eigen scalar.

 Instantiated templates for the following ScalarTypes are provided:
    - double
    - AutoDiffXd
 */
template <typename T>
class PointContactInfo {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PointContactInfo)

  /**
   Fully-specified constructor.

   @param application_point         The point at which the wrench is applied.
   @param normal                    The translational force's unit-length normal
                                    direction.
   @param force                     The translational force.
   @param torque                    The torque component.
   */
  PointContactInfo(
      const Vector3<T>& nhat_BA, const Vector3<T>& that,
      const T& phi, const T& vn, const T& vt, const Vector3<T>& v_AcBc,
      const Vector3<T>& vt_AcBc, const T& fn_AC, const T& ft_AC,
      const T& mu_stribeck);

 private:
  // Normal ᴮn̂ᴬ pointing out from body B and into body A, expressed in world
  // frame W.
  Vector3<T> nhat_BA_W_{};
  // Tangent vector t̂ to the surfaces at the contact point.
  Vector3<T> that_W_{};
  // Penetration distance.
  T phi_;
  // Normal velocity magnitude, ẋ = vₙ.
  T vn_{};
  // Tangential velocity magnitude.
  T vt_{};
  // Velocity of point Bc (contact point C instantaneously moving with body B)
  // relative to point Ac (contact point C instantaneously moving with body A),
  // expressed in world. That is, ᴬᶜvᴮᶜ = vₙ n̂ + vₜ t̂.
  Vector3<T> v_AcBc_W_{};
  // Tangential velocity component of the velocity of body B in A, expressed
  // in world. That is, the projection of v_AcBc_W into the tangent plane.
  Vector3<T> vt_AcBc_W_{};
  // Magnitude of the normal force on A applied at C.
  // Normal force vector is fn_AC_W = fn_AC * nhat_BA_W.
  T fn_AC_;
  // Magnitude of the tangential (friction) force on A applied at C.
  // Tangential force vector is ft_AC_W = ft_AC * that.
  T ft_AC_;
  // Stribeck friction coefficient for the current tangential speed vt.
  T mu_stribeck_;
};

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake
