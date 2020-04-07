#pragma once

#include <cmath>
#include <tuple>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/quaternion.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace free_body {

/// The purpose of the %FreeBody class is to provide the data (initial values
/// and gravity) and methods for calculating the exact analytical solution for
/// the translational and rotational motion of a torque-free rigid body B with
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

  /// Constructs a class that can be queried for exact values of orientation,
  /// position, and motion of a torque-free rigid body at time t.
  /// @param[in] initial_quat_NB  Value at time t = 0 of the quaternion
  /// relating right-handed orthonormal vectors Nx, Ny, Nz fixed in N (world)
  /// to right-handed orthonormal unit vectors Bx, By, Bz fixed in B (body).
  /// Note: The unit vector Bz is parallel to body B's symmetry axis.
  /// Note: The quaternion should already be normalized before it is passed.
  /// @param[in] initial_W_NB_B Value at time t = 0 of the angular velocity in N
  /// of body B, expressed in N.
  /// @param[in] initial_p_NoBcm_N Value at time t = 0 of the position vector
  /// from No (origin of world N) to Bcm (B's center of mass), expressed in N.
  /// @param[in] initial_v_NBcm_N Value at time t = 0 of the velocity in N
  /// of Bcm (B's center of mass), expressed in N.
  /// @param[in] gravity_N Local gravitational acceleration, expressed in N.
  FreeBody(const Eigen::Quaterniond& initial_quat_NB,
           const Eigen::Vector3d& initial_w_NB_B,
           const Eigen::Vector3d& initial_p_NoBcm_N,
           const Eigen::Vector3d& initial_v_NBcm_B,
           const Eigen::Vector3d& gravity_N)
      : initial_quat_NB_(initial_quat_NB),
        initial_w_NB_B_(initial_w_NB_B),
        initial_p_NoBcm_N_(initial_p_NoBcm_N),
        initial_v_NBcm_B_(initial_v_NBcm_B),
        uniform_gravity_expressed_in_world_(gravity_N) {}

  ~FreeBody() = default;

  /// Returns body B's moment of inertia about any axis that passes through Bcm
  /// (B's center of mass) and is perpendicular to B's inertia symmetry axis.
  /// For example, for a cylinder of radius r, length h and uniformly
  /// distributed mass m with its cylindrical axis aligned along its body frame
  /// z-axis this would be: I = Ixx = Iyy = m / 12 (3 r² + h²)
  double get_I() const { return 0.04; }

  /// Returns body's moment of inertia about the axis that passes through Bcm
  /// (B's center of mass) and is parallel to B's inertia symmetry axis.
  /// For example, for a cylinder of radius r, length h and uniformly
  /// distributed mass  m with its cylindrical axis aligned along its body frame
  /// z-axis this would be: J = Izz = m r² / 2
  double get_J() const { return 0.02; }

  // Get methods for initial values and gravity.
  const Eigen::Quaterniond& get_initial_quat_NB() const {
    return initial_quat_NB_;
  }
  const Eigen::Vector3d& get_initial_w_NB_B() const { return initial_w_NB_B_; }
  Eigen::Vector3d CalcInitial_w_NB_N() const {
    const math::RotationMatrixd R_NB(initial_quat_NB_);
    return R_NB * initial_w_NB_B_;
  }
  const Eigen::Vector3d& get_initial_p_NoBcm_N() const {
    return initial_p_NoBcm_N_;
  }
  const Eigen::Vector3d& get_uniform_gravity_expressed_in_world() const {
    return uniform_gravity_expressed_in_world_;
  }
  Eigen::Vector3d CalcInitial_v_NBcm_N() const {
    const math::RotationMatrixd R_NB(initial_quat_NB_);
    return R_NB * initial_v_NBcm_B_;
  }

  // Set methods for initial values and gravity.
  void set_initial_quat_NB(const Eigen::Quaterniond& quat_NB) {
    initial_quat_NB_ = quat_NB;
  }
  void set_initial_w_NB_B(const Eigen::Vector3d& w_NB_B) {
    initial_w_NB_B_ = w_NB_B;
  }
  void set_initial_p_NoBcm_N(const Eigen::Vector3d& p_NoBcm_N) {
    initial_p_NoBcm_N_ = p_NoBcm_N;
  }
  void set_initial_v_NBcm_B(const Eigen::Vector3d& v_NBcm_B) {
    initial_v_NBcm_B_ = v_NBcm_B;
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

  /// Returns angular rates associated with spin `s` and precession `p` from the
  /// analytical solution [Kane, 1983] for rotational motion (angular velocity
  /// and quaternion) for torque-free motion of an axis-symmetric rigid body B
  /// in a Newtonian frame (World).  Kane's solution for B's angular velocity
  /// `wx*Bx + wy*By + wz*Bz` is in terms of initial values wx0, wy0, wz0 as
  /// wx =  wx0 * cos(s * t) + wy0 * sin(s * t)
  /// wy = -wx0 * sin(s * t) + wy0 * cos(s * t)
  /// wz =  wz0
  /// For more information, see [Kane, 1983] Pages 60-62 and 159-169.
  /// @note the return value of `s` may be negative, zero, or positive, whereas
  ///       the return value of `p` is nonnegative.  The values of `s` and `p`
  ///       are returned in units of radian/second.
  ///
  /// - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York,
  ///   1983. (with P. W. Likins and D. A. Levinson).  Available for free .pdf
  ///   download: https:///ecommons.cornell.edu/handle/1813/637
  std::pair<double, double> CalcAngularRates_s_p() const {
    const double I = get_I();
    const double J = get_J();
    const Vector3<double>& initial_w_NB_B = get_initial_w_NB_B();
    const double wx0 = initial_w_NB_B[0];
    const double wy0 = initial_w_NB_B[1];
    const double wz0 = initial_w_NB_B[2];
    const double s = (I - J) / I * wz0;
    const double z = wz0 * J / I;
    const double p =  std::sqrt(wx0 * wx0 + wy0 * wy0 + z * z);
    return std::make_pair(s, p);
  }

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

  // Initial (t = 0) value of the quaternion relating unit vectors Nx, Ny, Nz
  // fixed in World N to unit vectors Bx, By, Bz fixed in body B (Bz is parallel
  // to body B's symmetry axis). Note: The quaternion notation quat_NB is
  // analogous to the rotation matrix notation R_NB.
  Eigen::Quaterniond initial_quat_NB_;

  // Initial (t = 0) value of angular velocity in N of B, expressed in B.
  Eigen::Vector3d initial_w_NB_B_;

  // Initial (t = 0) value of the position vector from No (world origin) to
  // Bcm (B's center of mass), expressed in N.
  Eigen::Vector3d initial_p_NoBcm_N_;

  // Initial (t = 0) value of the velocity in N of Bcm, expressed in B.
  // Note: v_NBcm_B is not the time-derivative in B (or N) of p_NoBcm_N_.
  Eigen::Vector3d initial_v_NBcm_B_;

  // uniform_gravity_expressed_in_world_ is the local planet's (e.g., Earth)
  // uniform gravitational acceleration, expressed in World (e.g., Earth).
  Eigen::Vector3d uniform_gravity_expressed_in_world_;
};

}  // namespace free_body
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
