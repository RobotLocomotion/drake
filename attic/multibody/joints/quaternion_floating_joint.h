#pragma once

#include <memory>
#include <string>

#include "drake/common/constants.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/math/normalize_vector.h"
#include "drake/math/quaternion.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_conversion_gradient.h"
#include "drake/multibody/joints/drake_joint_impl.h"
#include "drake/util/drakeGeometryUtil.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
// TODO(sherm1,mitiguy) Verify that this is correct.
/**
 * Defines a 6 dof tree joint (mobilizer) that uses a unit quaternion as the
 * generalized orientation coordinates.
 *
 * <h3>Generalized coordinates (configuration variables)</h3>
 * There are 7 generalized coordinates q,
 * organized as a position vector and quaternion. A tree joint connects an
 * inboard (parent) body P to an outboard (child) body B. In those terms this
 * joint's generalized coordinates are: <pre>
 *          --------- ------------- T
 *     q = | p_PB_P  |    q_PB     |
 *          --------- -------------  7×1
 *          px py pz   qw qx qy qz
 * </pre>
 * where `p_PB_P` is the position vector from P's origin Po to B's origin Bo,
 * expressed in the P basis, and `q_PB` is the quaternion that is equivalent
 * to the rotation matrix `R_PB`. The second line shows the 7 generalized
 * coordinate scalars in order. Note that `qw` is the scalar part of the
 * quaternion while `[qx qy qz]` is the vector part. See
 * @ref multibody_spatial_pose for more  information about this notation.
 *
 * The time derivatives qdot of the generalized coordinates, _not_ to be
 * confused with the generalized velocity variables v, are: <pre>
 *          --------- ------------- T
 *  qdot = | v_PB_P  |   qdot_PB   |
 *          --------- -------------  7×1
 * </pre>
 * where `v_PB_P = d_P/dt p_PB_P` is the velocity of point Bo measured and
 * expressed in the P frame, where we have emphasized that the derivative is
 * taken in P, and `qdot_PB = d/dt q_PB` is the time derivative of the
 * quaternion.
 *
 * <h3>Generalized velocity</h3>
 * There are 6 generalized velocity variables v, organized as follows: <pre>
 *          --------- -------- T
 *     v = | ω_PB_B  | v_PB_B |
 *          --------- --------  6×1
 * </pre>
 * where `ω_PB_B` is B's angular velocity in P, expressed in B, and
 * `v_PB_B` is point Bo's translational velocity in P, expressed in B.
 *
 * Note that
 * the rotational and translational quantities are in the reverse order from
 * those in the generalized coordinates, and that the velocities are expressed
 * in the _child_ frame B rather than in the parent. Clearly these are _not_
 * the derivatives of the generalized coordinates!
 *
 * The time derivatives of the generalized velocities are: <pre>
 *          --------- ----------- T
 *  vdot = | α_PB_B  | vdot_PB_B |
 *          --------- -----------  6×1
 * </pre>
 * where `α_PB_B` is B's angular acceleration in P, expressed in B, and
 * `vdot_PB_B = d_B/dt v_PB_B` where we have emphasized that the derivative is
 * taken in the B frame, so this is *not* the acceleration of Bo in P. That
 * acceleration is given by `a_PB_B = vdot_PB_B + ω_PB_B × v_PB_B` (still in B).
 * Re-expressing `a_PB_B` in P provides the configuration second derivative
 * `a_PB_P = d²_P/dt² p_PB_P = R_PB*a_PB_B`.
 */
class QuaternionFloatingJoint : public DrakeJointImpl<QuaternionFloatingJoint> {
 public:
  QuaternionFloatingJoint(const std::string& name,
                          const Eigen::Isometry3d& transform_to_parent_body)
      : DrakeJointImpl(*this, name, transform_to_parent_body, 7, 6) {}

  virtual ~QuaternionFloatingJoint() {}

  /** Returns the transform `X_PB(q)` where P is the parent body and B the
   * child body connected by this joint.
   */
  template <typename DerivedQ>
  Eigen::Transform<typename DerivedQ::Scalar, 3, Eigen::Isometry>
  jointTransform(const Eigen::MatrixBase<DerivedQ>& q) const {
    const drake::Vector3<typename DerivedQ::Scalar> p_PoBo(q[0], q[1], q[2]);
    const drake::Vector4<typename DerivedQ::Scalar> wxyz =
        q.template bottomRows<4>();
    const Eigen::Quaternion<typename DerivedQ::Scalar> quaternion_PB(
        wxyz(0), wxyz(1), wxyz(2), wxyz(3));
    const drake::math::RigidTransform<typename DerivedQ::Scalar> X_PB(
        quaternion_PB, p_PoBo);
    return X_PB.GetAsIsometry3();
  }

  template <typename DerivedQ, typename DerivedMS>
  void motionSubspace(
      const Eigen::MatrixBase<DerivedQ>& q,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::MatrixBase<DerivedMS>& motion_subspace,
      typename drake::math::Gradient<DerivedMS, Eigen::Dynamic>::type*
          dmotion_subspace = nullptr) const {
    drake::unused(q);
    motion_subspace.setIdentity(drake::kTwistSize, get_num_velocities());
    if (dmotion_subspace) {
      dmotion_subspace->setZero(motion_subspace.size(), get_num_positions());
    }
  }

  template <typename DerivedQ, typename DerivedV>
  void motionSubspaceDotTimesV(
      const Eigen::MatrixBase<DerivedQ>& q,
      const Eigen::MatrixBase<DerivedV>& v,
      Eigen::Matrix<typename DerivedQ::Scalar, 6, 1>&
          motion_subspace_dot_times_v,
      typename drake::math::Gradient<
          Eigen::Matrix<typename DerivedQ::Scalar, 6, 1>, Eigen::Dynamic>::type*
          dmotion_subspace_dot_times_vdq = nullptr,
      typename drake::math::Gradient<
          Eigen::Matrix<typename DerivedQ::Scalar, 6, 1>, Eigen::Dynamic>::type*
          dmotion_subspace_dot_times_vdv = nullptr) const {
    drake::unused(q, v);
    motion_subspace_dot_times_v.setZero();
    if (dmotion_subspace_dot_times_vdq) {
      dmotion_subspace_dot_times_vdq->setZero(
          motion_subspace_dot_times_v.size(), get_num_positions());
    }
    if (dmotion_subspace_dot_times_vdv) {
      dmotion_subspace_dot_times_vdv->setZero(
          motion_subspace_dot_times_v.size(), get_num_velocities());
    }
  }

  /**
   * For the %QuaternionFloatingJoint, computes the matrix `N⁺(q)`∊ℝ⁶ˣ⁷ that
   * maps generalized coordinate time derivatives qdot to generalized
   * velocities v, with `v=N⁺ qdot`. The name signifies that `N⁺=pinv(N)` where
   * `N(q)` is the matrix that maps v to qdot with `qdot=N v` and `pinv()`
   * is the pseudoinverse (in this case the left pseudoinverse).
   *
   * See the class description for precise definitions of the generalized
   * coordinates and velocities. Because the velocities are not the time
   * derivatives of the coordinates, rotations and translations are
   * reversed, and different expressed-in frames are employed, `N⁺` has the
   * following elaborate structure: <pre>
   *        -------- -----------
   *       |  0₃ₓ₃  | Nq⁺_PB_B  |
   *  N⁺ = |--------|-----------|
   *       |  R_BP  |   0₃ₓ₄    |
   *        -------- ----------- 6×7
   * </pre>
   * where `Nq_PB_B` is the matrix that maps angular velocity `ω_PB_B` to
   * quaternion time derivative `qdot_PB` such that `qdot_PB=Nq_PB_B*ω_PB_B`,
   * and `Nq⁺_PB_B` is the left pseudoinverse of `Nq_PB_B`.
   *
   * @param[in]  q The 7-element generalized configuration variable. See the
   *   class documentation for details. See warning below regarding the effect
   *   if the contained quaternion is not normalized.
   * @param[out] qdot_to_v The matrix `N⁺`.
   * @param      dqdot_to_v Unused, must be `nullptr` on entry.
   *
   * @warning Let `s` be the norm of the quaternion in `q`. If `s ≠ 1`, then
   * we will calculate `s*Nq⁺_PB_B` in the upper right block of `N⁺` so the
   * resulting angular velocity vector will be scaled by `s` as well. This
   * method neither performs a normalization check nor normalizes the quaternion
   * orientation parameters. Implications for integration techniques must be
   * carefully considered.
   */
  template <typename DerivedQ>
  void qdot2v(const Eigen::MatrixBase<DerivedQ>& q,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic,
                            Eigen::Dynamic, 0, DrakeJoint::MAX_NUM_VELOCITIES,
                        // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                            DrakeJoint::MAX_NUM_POSITIONS>& qdot_to_v,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic,
                            Eigen::Dynamic>* dqdot_to_v) const {
    if (dqdot_to_v) {
      throw std::runtime_error("no longer supported");
    }

    qdot_to_v.resize(get_num_velocities(), get_num_positions());

    // Get the quaternion values.
    auto quaternion_block =
        q.template middleRows<drake::kQuaternionSize>(drake::kSpaceDimension);
    const typename DerivedQ::Scalar& ew = quaternion_block[0];
    const typename DerivedQ::Scalar& ex = quaternion_block[1];
    const typename DerivedQ::Scalar& ey = quaternion_block[2];
    const typename DerivedQ::Scalar& ez = quaternion_block[3];

    // Assume that the quaternion orientation (e) gives a transformation matrix
    // that re-expresses vectors from some frame B to some frame N. The upper
    // right hand block of the transformation matrix represents
    // the 2 L part of the relationship ω = 2 L de/dt, where
    // e = [ ew ex ey ez ] are the quaternion values and ω is an angular
    // velocity given in frame B. The relationship ω = 2 L de/dt and the
    // matrix L was taken from:
    // - P. Nikravesh, Computer-Aided Analysis of Mechanical Systems. Prentice
    //     Hall, New Jersey, 1988. Equation 6.108.
    // NOTE: the torque-free, cylindrical solid unit test successfully detects
    //       when this matrix (incorrectly) is set to that which transforms
    //       unit quaternion time derivatives to angular velocities in the
    //       global frame.
    qdot_to_v.template block<3, 3>(0, 0).setZero();
    qdot_to_v.template block<3, 4>(0, 3) <<  -ex,  ew,  ez, -ey,
                                             -ey, -ez,  ew,  ex,
                                             -ez,  ey, -ex,  ew;
    qdot_to_v.template block<3, 4>(0, 3) *= 2.;

    // Given the arbitrary frames B and N described above, the next three
    // columns re-express linear velocities from frame N to frame B.
    const Eigen::Quaternion<typename DerivedQ::Scalar> quat(ew, ex, ey, ez);
    const drake::math::RotationMatrix<typename DerivedQ::Scalar> R(quat);
    qdot_to_v.template block<3, 3>(3, 0) = R.inverse().matrix();
    qdot_to_v.template block<3, 4>(3, 3).setZero();
  }


  /**
   * For the %QuaternionFloatingJoint, computes the matrix `N(q)`∊ℝ⁷ˣ⁶ that
   * maps generalized velocities v to generalized coordinate time derivatives
   * qdot, with `qdot=N v`.
   *
   * See the class description for precise definitions of the generalized
   * coordinates and velocities. Because the velocities are not the time
   * derivatives of the coordinates, rotations and translations are
   * reversed, and different expressed-in frames are employed, `N` has the
   * following elaborate structure: <pre>
   *        ------- ------
   *       |  0₃ₓ₃ | R_PB |
   *       |-------|------|
   *   N = |       |      |
   *       |Nq_PB_B| 0₄ₓ₃ |
   *       |       |      |
   *        -------------- 7×6
   * </pre>
   * where `Nq_PB_B` is the matrix that maps angular velocity `ω_PB_B` to
   * quaternion time derivative `qdot_PB` such that `qdot_PB=Nq_PB_B*ω_PB_B`.
   *
   * @param[in]  q The 7-element generalized configuration variable. See the
   *   class documentation for details. See warning below regarding the effect
   *   if the contained quaternion is not normalized.
   * @param[out] v_to_qdot The matrix `N`.
   * @param      dv_to_qdot Unused, must be `nullptr` on entry.
   *
   * @warning Let `s` be the norm of the quaternion in `q`. If `s ≠ 1`, then
   * we will calculate `s*Nq_PB_B` in the lower left block of `N` so the
   * resulting quaternion derivative will be scaled by `s` as well. This
   * method neither performs a normalization check nor normalizes the quaternion
   * orientation parameters. Implications for integration techniques must be
   * carefully considered.
   */
  template <typename DerivedQ>
  void v2qdot(const Eigen::MatrixBase<DerivedQ>& q,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic,
                            Eigen::Dynamic, 0, DrakeJoint::MAX_NUM_POSITIONS,
                        // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                            DrakeJoint::MAX_NUM_VELOCITIES>& v_to_qdot,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic,
                            Eigen::Dynamic>* dv_to_qdot) const {
    v_to_qdot.resize(get_num_positions(), get_num_velocities());

    if (dv_to_qdot) {
      throw std::runtime_error("no longer supported");
    }

    // Get the quaternion values.
    auto quaternion_block =
        q.template middleRows<drake::kQuaternionSize>(drake::kSpaceDimension);
    const typename DerivedQ::Scalar& ew = quaternion_block[0];
    const typename DerivedQ::Scalar& ex = quaternion_block[1];
    const typename DerivedQ::Scalar& ey = quaternion_block[2];
    const typename DerivedQ::Scalar& ez = quaternion_block[3];

    // Assume that the quaternion orientation (e) gives a transformation matrix
    // that re-expresses vectors from some frame B to some frame N. The upper
    // right hand block of the tranfsormation matrix corresponds to the
    // transpose of the "L" matrix used in qdot2v(). Specifically, this matrix
    // represents the 1/2 Lᵀ part of the relationship de/dt = 1/2 Lᵀω, where
    // e = [ ew ex ey ez ] are the quaternion values and ω is an angular
    // velocity given in frame B. The relationship de/dt = 1/2 Lᵀω and the
    // matrix L was taken from:
    // - P. Nikravesh, Computer-Aided Analysis of Mechanical Systems. Prentice
    //     Hall, New Jersey, 1988. Equation 6.109.
    v_to_qdot.template block<4, 3>(0, 0).setZero();
    v_to_qdot.template block<4, 3>(3, 0) <<  -ex, -ey, -ez,
                                              ew, -ez,  ey,
                                              ez,  ew, -ex,
                                             -ey,  ex,  ew;
    v_to_qdot.template block<4, 3>(3, 0) *= 0.5;

    // Given the arbitrary frames B and N described above, the next three
    // columns re-express linear velocities from frame B to frame N.
    const Eigen::Quaternion<typename DerivedQ::Scalar> quat(ew, ex, ey, ez);
    const drake::math::RotationMatrix<typename DerivedQ::Scalar> R(quat);
    v_to_qdot.template block<3, 3>(0, 3) = R.matrix();
    v_to_qdot.template block<4, 3>(3, 3).setZero();
  }

  template <typename DerivedV>
  Eigen::Matrix<typename DerivedV::Scalar, Eigen::Dynamic, 1> frictionTorque(
      const Eigen::MatrixBase<DerivedV>& v) const {
    drake::unused(v);
    return Eigen::Matrix<typename DerivedV::Scalar, Eigen::Dynamic, 1>::Zero(
        get_num_velocities(), 1);
  }

  template <typename DerivedQ>
  Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, 1> SpringTorque(
      const Eigen::MatrixBase<DerivedQ>& q) const {
    drake::unused(q);
    // Returning zero for now, but a 3D torsional spring could theoretically be
    // added here.
    return drake::VectorX<typename DerivedQ::Scalar>::Zero(
        get_num_velocities(), 1);
  }


  bool is_floating() const override { return true; };

  std::string get_position_name(int index) const override;
  std::string get_velocity_name(int index) const override;
  Eigen::VectorXd zeroConfiguration() const override;
  Eigen::VectorXd randomConfiguration(
      std::default_random_engine& generator) const override;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  std::unique_ptr<DrakeJoint> DoClone() const final;
  void DoInitializeClone(DrakeJoint*) const final {}
};
#pragma GCC diagnostic pop  // pop -Wno-overloaded-virtual
