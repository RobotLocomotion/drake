#pragma once

#include "drake/common/eigen_types.h"
#include "drake/math/quaternion.h"

namespace drake {
namespace multibody {

template <typename T>
class QuaternionRate {
 public:
  static Eigen::Matrix<T, 4, 3> AngularVelocityToQuaternionRateMatrix(
      const Quaternion<T>& q_FM) {
    // With L given by CalcLMatrix we have:
    // N(q) = L(q_FM/2)
    return CalcLMatrix(
        {q_FM.w() / 2.0, q_FM.x() / 2.0, q_FM.y() / 2.0, q_FM.z() / 2.0});
  }

  static Eigen::Matrix<T, 3, 4> QuaternionRateToAngularVelocityMatrix(
      const Quaternion<T>& q_FM) {
    const T q_norm = q_FM.norm();
    // The input quaternion might not be normalized. We refer to the normalized
    // quaternion as q_FM_tilde. This is retrieved as a Vector4 with its storage
    // order consistent with the storage order in a MultibodyPlant context. That
    // is, scalar component first followed by the vector component. See developers
    // notes in the implementation for get_quaternion().
    const Vector4<T> q_FM_tilde =
        Vector4<T>(q_FM.w(), q_FM.x(), q_FM.y(), q_FM.z()) / q_norm;

    // Gradient of the normalized quaternion with respect to the unnormalized
    // generalized coordinates:
    const Matrix4<T> dqnorm_dq =
        (Matrix4<T>::Identity() - q_FM_tilde * q_FM_tilde.transpose()) / q_norm;

    // With L given by CalcLMatrix we have:
    // N⁺(q_tilde) = L(2 q_FM_tilde)ᵀ
    return CalcLMatrix({2.0 * q_FM_tilde[0], 2.0 * q_FM_tilde[1],
                        2.0 * q_FM_tilde[2], 2.0 * q_FM_tilde[3]})
        .transpose() * dqnorm_dq;
  }

 private:
  static Eigen::Matrix<T, 4, 3> CalcLMatrix(
      const Quaternion<T>& q_FM) {
    // This L matrix helps us compute both N(q) and N⁺(q) since it turns out that:
    //   N(q) = L(q_FM/2)
    // and:
    //   N⁺(q) = L(2 q_FM)ᵀ
    // See Eqs. 5 and 6 in Section 9.2 of Paul's book
    // [Mitiguy (August 7) 2017, §9.2], for the time derivative of the vector
    // component of the quaternion (Euler parameters). Notice however here we use
    // qs and qv for the "scalar" and "vector" components of the quaternion q_FM,
    // respectively, while Mitiguy uses ε₀ and ε (in bold), respectively.
    // This mobilizer is parameterized by the angular velocity w_FM, i.e. time
    // derivatives of the vector component of the quaternion are taken in the F
    // frame. If you are confused by this, notice that the vector component of a
    // quaternion IS a vector, and therefore you must specify in what frame time
    // derivatives are taken.
    //
    // Notice this is equivalent to:
    // Dt_F(q) = 1/2 * w_FM⋅q_FM, where ⋅ denotes the "quaternion product" and
    // both the vector component qv_FM of q_FM and w_FM are expressed in frame F.
    // Dt_F(q) is short for [Dt_F(q)]_F.
    // The expression above can be written as:
    // Dt_F(q) = 1/2 * (-w_FM.dot(qv_F); qs * w_FM + w_FM.cross(qv_F))
    //         = 1/2 * (-w_FM.dot(qv_F); qs * w_FM - qv_F.cross(w_FM))
    //         = 1/2 * (-w_FM.dot(qv_F); (qs * Id - [qv_F]x) * w_FM)
    //         = L(q_FM/2) * w_FM
    // That is:
    //        |         -qv_Fᵀ    |
    // L(q) = | qs * Id - [qv_F]x |

    const T qs = q_FM.w();             // The scalar component.
    const Vector3<T> qv = q_FM.vec();  // The vector component.
    const Vector3<T> mqv = -qv;        // minus qv.

    // NOTE: the rows of this matrix are in an order consistent with the order
    // in which we store the quaternion in the state, with the scalar component
    // first followed by the vector component.
    return (Eigen::Matrix<T, 4, 3>() << mqv.transpose(), qs, qv.z(), mqv.y(),
            mqv.z(), qs, qv.x(), qv.y(), mqv.x(), qs)
        .finished();
  }

};

}  // namespace multibody
}  // namespace drake
