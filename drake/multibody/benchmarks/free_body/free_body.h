#pragma once

#include <cmath>
#include <tuple>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/quaternion.h"

namespace drake {
namespace benchmarks {
namespace free_body {

/// The purpose of the %FreeBody class is to provide the data (initial values
/// and gravity) and methods for calculating the exact analytical solution for
/// the translational and rotational motion of a toque-free rigid body B with
/// axially symmetric inertia, in a Newtonian frame (World) N. Examples of
/// bodies with axially symmetric inertia include cylinders, rods or bars with a
/// circular or square cross section and spinning tops.
/// Since the only external forces on B are uniform gravitational forces, there
/// exists an exact closed-form analytical solution for B's motion. The closed-
/// form rotational solution is available since B is "torque-free", i.e., the
/// moment of all forces about B's mass center is zero.
/// This class calculates the body B's quaternion, angular velocity and angular
/// acceleration expressed in B (body-frame) as well as the position, velocity,
/// acceleration of Bcm (B's center of mass) in N (World).
/// Algorithm from [Kane, 1983] Sections 1.13 and 3.1, Pages 60-62 and 159-169.
///
/// - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
///   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
///   https:///ecommons.cornell.edu/handle/1813/637
class FreeBody {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FreeBody)

  FreeBody(const Eigen::Quaterniond& quat_NB_initial,
           const Eigen::Vector3d& w_NB_B_initial,
           const Eigen::Vector3d& p_NoBcm_N_initial,
           const Eigen::Vector3d& v_NBcm_B_initial,
           const Eigen::Vector3d& gravity_N)
      : quat_NB_initial_(quat_NB_initial),
        w_NB_B_initial_(w_NB_B_initial),
        p_NoBcm_N_initial_(p_NoBcm_N_initial),
        v_NBcm_B_initial_(v_NBcm_B_initial),
        uniform_gravity_expressed_in_world_(gravity_N) {}

  ~FreeBody() = default;

  /// Returns the body's moment of inertia about an axis perpendicular to its
  /// axis of rotation and passing through its center of mass.
  /// For example, for a cylinder of radius r, length h and uniformly
  /// distributed mass m with its rotational axis aligined along its body frame
  /// z-axis this would be: <pre>
  ///   I = Ixx = Iyy = m / 12 (3 r² + h²)
  /// </pre>
  double get_I() const { return 0.04; }

  /// Returns body's moment of inertia about its axis of rotation.
  /// For example, for a cylinder of radius r, length h and uniformly
  /// distributed mass  m with its rotational axis aligined along its body frame
  /// z-axis this would be: <pre>
  ///   J = Izz = m r² / 2
  /// </pre>
  double get_J() const { return 0.02; }

  // Get methods for initial values and gravity.
  const Eigen::Quaterniond& get_quat_NB_initial() const {
    return quat_NB_initial_;
  }
  const Eigen::Vector3d& get_w_NB_B_initial() const { return w_NB_B_initial_; }
  const Eigen::Vector3d& get_p_NoBcm_N_initial() const {
    return p_NoBcm_N_initial_;
  }
  const Eigen::Vector3d& get_v_NBcm_B_initial() const {
    return v_NBcm_B_initial_;
  }
  const Eigen::Vector3d& get_uniform_gravity_expressed_in_world() const {
    return uniform_gravity_expressed_in_world_;
  }
  Eigen::Vector3d GetInitialVelocityOfBcmInWorldExpressedInWorld() const {
    const Eigen::Matrix3d R_NB_initial = quat_NB_initial_.toRotationMatrix();
    return R_NB_initial * v_NBcm_B_initial_;
  }

  // Set methods for initial values and gravity.
  void set_quat_NB_initial(const Eigen::Quaterniond& quat_NB_initial) {
    quat_NB_initial_ = quat_NB_initial;
  }
  void set_w_NB_B_initial(const Eigen::Vector3d& w_NB_B_initial) {
    w_NB_B_initial_ = w_NB_B_initial;
  }
  void set_p_NoBcm_N_initial(const Eigen::Vector3d& p_NoBcm_N_initial) {
    p_NoBcm_N_initial_ = p_NoBcm_N_initial;
  }
  void set_v_NBcm_B_initial(const Eigen::Vector3d& v_NBcm_B_initial) {
    v_NBcm_B_initial_ = v_NBcm_B_initial;
  }
  void SetUniformGravityExpressedInWorld(const Eigen::Vector3d& gravity) {
    uniform_gravity_expressed_in_world_ = gravity;
  }

  /// Calculates exact solutions for quaternion and angular velocity expressed
  /// in body-frame, and their time derivatives for torque-free rotational
  /// motion of axis-symmetric rigid body B in Newtonian frame (World) N,
  /// where torque-free means the moment of forces about B's mass center is
  /// zero. The quaternion characterizes the orientation between
  /// right-handed orthogonal unit vectors Nx, Ny, Nz fixed in N and
  /// right-handed orthogonal unit vectors Bx, By, Bz fixed in B, where Bz is
  /// parallel to B's symmetry axis.
  ///
  /// @note CalculateExactRotationalSolutionABInitiallyAligned() implements the
  /// algorithm from [Kane, 1983] Sections 1.13 and 3.1, Pages 60-62 and
  /// 159-169.
  ///
  /// @param t Current value of time.
  /// @returns Machine-precision values at time t are returned as defined below.
  ///
  /// @note This function allows for initial misalignment of Nx, Ny, Nz and
  /// Bx, By, Bz.
  ///
  /// std::tuple | Description
  /// -----------|-------------------------------------------------
  /// quat_NB    | Quaternion relating frame N to frame B: [e0, e1, e2, e3]
  ///            | Note: quat_NB is analogous to the rotation matrix R_NB.
  /// quatDt     | Time-derivative of `quat_NB', i.e., [ė0, ė1, ė2, ė3].
  /// w_NB_B     | B's angular velocity in N, expressed in B.
  /// alpha_NB_B | B's angular acceleration in N, expressed in B.
  ///
  /// - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York,
  ///   1983. (with P. W. Likins and D. A. Levinson).  Available for free .pdf
  ///   download: https:///ecommons.cornell.edu/handle/1813/637
  std::tuple<Eigen::Quaterniond, Eigen::Vector4d, Eigen::Vector3d,
             Eigen::Vector3d>
  CalculateExactRotationalSolutionNB(const double t) const;

  /// Calculates exact solutions for translational motion of an arbitrary rigid
  /// body B in a Newtonian frame (world) N.  Algorithm from high-school
  /// physics.
  ///
  /// @param t Current value of time.
  /// @returns Machine-precision values at time t are returned as defined below.
  ///
  /// std::tuple | Description
  /// -----------|-----------------------------------------------------------
  /// xyz        | Vector3d [x, y, z], Bcm's position from No, expressed in N.
  /// xyzDt      | Vector3d [ẋ, ẏ, ż]  Bcm's velocity in N, expressed in N.
  /// xyzDDt     | Vector3d [ẍ  ÿ  z̈], Bcm's acceleration in N, expressed in N.
  std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
  CalculateExactTranslationalSolution(const double t) const;

 private:
  // This "helper" method calculates quat_NB, w_NB_B, and alpha_NB_B at time t.
  // More precisely, this method calculates exact solutions for quaternion,
  // angular velocity, and angular acceleration expressed in body-frame, for
  // torque-free rotational motion of an axis-symmetric rigid body B in
  // Newtonian frame (World) N, where torque-free means the moment of forces
  // about B's mass center is zero.
  // Right-handed orthogonal unit vectors Nx, Ny, Nz fixed in N are initially
  // equal to right-handed orthogonal unit vectors Bx, By, Bz fixed in B,
  // where Bz is parallel to B's symmetry axis.
  // Note: The function CalculateExactRotationalSolutionNB() is a more general
  // solution that allows for initial misalignment of Bx, By, Bz.
  // Algorithm from [Kane, 1983] Sections 1.13 and 3.1, Pages 60-62 and 159-169.
  // @param t Current value of time.
  // @returns Machine-precision values at time t are returned as defined below.
  //
  // std::tuple | Description
  // -----------|-------------------------------------------------
  // quat_NB    | Quaternion relating Nx, Ny, Nz to Bx, By, Bz.
  //            | Note: quat_NB is analogous to the rotation matrix R_NB.
  // w_NB_B     | B's angular velocity in N, expressed in B, e.g., [wx, wy, wz].
  // alpha_NB_B | B's angular acceleration in N, expressed in B, [ẇx, ẇy, ẇz].
  //
  // - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
  //   (with P. W. Likins and D. A. Levinson).  Available for free .pdf
  // download: https://ecommons.cornell.edu/handle/1813/637
  std::tuple<Eigen::Quaterniond, Eigen::Vector3d, Eigen::Vector3d>
  CalculateExactRotationalSolutionABInitiallyAligned(const double t) const;

  // quat_NB_initial_ is the initial (t=0) value of the quaternion that relates
  // unit vectors Nx, Ny, Nz fixed in World N (e.g., Nz vertically upward) to
  // unit vectors Bx, By, Bz fixed in body B (Bz parallel to symmetry axis)
  // Note: The quaternion should already be normalized before it is set.
  // Note: quat_NB_initial is analogous to the initial rotation matrix R_NB.
  Eigen::Quaterniond quat_NB_initial_;

  // w_NB_B_initial_ is B's initial angular velocity in N, expressed in B.
  Eigen::Vector3d w_NB_B_initial_;

  // p_NoBcm_N_initial_ is Bcm's initial position from No, expressed in N, i.e.,
  // x, y, z, the Nx, Ny, Nz measures of Bcm's position vector from point No
  // (World origin).  Note: Bcm (B's center of mass) is coincident with Bo.
  Eigen::Vector3d p_NoBcm_N_initial_;

  // v_NBcm_B_initial_ is Bcm's initial velocity in N, expressed in B.
  // Note: v_NBcm_B is not (in general) the time-derivative of ẋ, ẏ, ż.
  Eigen::Vector3d v_NBcm_B_initial_;

  // uniform_gravity_expressed_in_world_ is the local planet's (e.g., Earth)
  // uniform gravitational acceleration, expressed in World (e.g., Earth).
  Eigen::Vector3d uniform_gravity_expressed_in_world_;
};

}  // namespace free_body
}  // namespace benchmarks
}  // namespace drake
