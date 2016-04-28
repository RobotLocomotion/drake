#ifndef DRAKE_ROLLPITCHYAWFLOATING_H
#define DRAKE_ROLLPITCHYAWFLOATING_H

#include "drake/systems/plants/joints/JointType.h"

namespace  Drake {

template <typename J>
class RollPitchYawFloating : public JointType<J> {
 public:
  static const int NUM_POSITIONS = 6;
  static const int NUM_VELOCITIES = 6;

  template <typename DerivedQ>
  Transform3D<Promote<J, typename DerivedQ::Scalar>> jointTransform(const Eigen::MatrixBase<DerivedQ> &q) const {
    using Q = typename DerivedQ::Scalar;
    using T = Promote<J, Q>;

    Transform3D<T> ret;
    ret.linear() = rpy2rotmat(ConvertMatrix<T>()(q.template middleRows<RPY_SIZE>(SPACE_DIMENSION)));
    ret.translation() = ConvertMatrix<T>()(q.template middleRows<SPACE_DIMENSION>(0));
    ret.makeAffine();
    return ret;
  }

  template <typename DerivedQ>
  MotionSubspace<Promote<J, typename DerivedQ::Scalar>> motionSubspace(const Eigen::MatrixBase<DerivedQ> &q) {
    using Q = typename DerivedQ::Scalar;
    using T = Promote<J, Q>;

    MotionSubspace<T> ret(TWIST_SIZE, RollPitchYawFloating<J>::NUM_VELOCITIES);
    auto rpy = ConvertMatrix<T>()(q.template middleRows<RPY_SIZE>(SPACE_DIMENSION));
    Eigen::Matrix<T, SPACE_DIMENSION, RPY_SIZE> E;
    rpydot2angularvelMatrix(rpy, E);
    Eigen::Matrix<T, 3, 3> R = rpy2rotmat(rpy);
    ret.template block<3, 3>(0, 0).setZero();
    ret.template block<3, 3>(0, 3) = R.transpose() * E;
    ret.template block<3, 3>(3, 0) = R.transpose();
    ret.template block<3, 3>(3, 3).setZero();
    return ret;
  }

  template <typename DerivedQ, typename DerivedV>
  SpatialVector<Promote<J, typename DerivedQ::Scalar>> motionSubspaceDotTimesV(const Eigen::MatrixBase<DerivedQ> &q, const Eigen::MatrixBase<DerivedV> &v) {
    using Q = typename DerivedQ::Scalar;
    using V = typename DerivedV::Scalar;
    static_assert(std::is_same<Q, V>::value, "type of velocity vector must match type of configuration vector");
    using T = Promote<J, Q>;

    SpatialVector<T> ret;

    auto rpy = ConvertMatrix<T>()(q.template middleRows<RPY_SIZE>(SPACE_DIMENSION));
    const T& roll = rpy(0);
    const T& pitch = rpy(1);
    const T& yaw = rpy(2);

    auto pd = ConvertMatrix<T>()(v.template middleRows<SPACE_DIMENSION>(0));
    const T& xd = pd(0);
    const T& yd = pd(1);
    const T& zd = pd(2);

    auto rpyd = ConvertMatrix<T>()(v.template middleRows<RPY_SIZE>(SPACE_DIMENSION));
    const T& rolld = rpyd(0);
    const T& pitchd = rpyd(1);
    const T& yawd = rpyd(2);

    T cr = cos(roll);
    T sr = sin(roll);
    T cp = cos(pitch);
    T sp = sin(pitch);
    T cy = cos(yaw);
    T sy = sin(yaw);

    ret.transpose() << -pitchd * yawd * cp,
        rolld * yawd * cp * cr - pitchd * yawd * sp * sr - pitchd * rolld * sr,
        -pitchd * rolld * cr - pitchd * yawd * cr * sp - rolld * yawd * cp * sr,
        yd * (yawd * cp * cy - pitchd * sp * sy) -
            xd * (pitchd * cy * sp + yawd * cp * sy) - pitchd * zd * cp,
        zd * (rolld * cp * cr - pitchd * sp * sr) +
            xd * (rolld * (sr * sy + cr * cy * sp) -
                yawd * (cr * cy + sp * sr * sy) + pitchd * cp * cy * sr) -
            yd * (rolld * (cy * sr - cr * sp * sy) +
                yawd * (cr * sy - cy * sp * sr) - pitchd * cp * sr * sy),
        xd * (rolld * (cr * sy - cy * sp * sr) +
            yawd * (cy * sr - cr * sp * sy) + pitchd * cp * cr * cy) -
            zd * (pitchd * cr * sp + rolld * cp * sr) +
            yd * (yawd * (sr * sy + cr * cy * sp) -
                rolld * (cr * cy + sp * sr * sy) + pitchd * cp * cr * sy);
    return ret;
  }

  template <typename Q>
  ConfigurationDerivativeToVelocity<Promote<J, Q>> configurationDerivativeToVelocity() {
    return ConfigurationDerivativeToVelocity<Promote<J, Q>>::Identity(RollPitchYawFloating<J>::NUM_VELOCITIES, RollPitchYawFloating<J>::NUM_POSITIONS);
  }

  template <typename Q>
  VelocityToConfigurationDerivative<Promote<J, Q>> velocityToConfigurationDerivative() {
    return VelocityToConfigurationDerivative<Promote<J, Q>>::Identity(RollPitchYawFloating<J>::NUM_POSITIONS, RollPitchYawFloating<J>::NUM_VELOCITIES);
  }

  template <typename V>
  Eigen::Matrix<Promote<J, V>, Eigen::Dynamic, 1> frictionTorque() {
    return Eigen::Matrix<V, Eigen::Dynamic, 1>::Zero(QuaternionFloating<J>::NUM_VELOCITIES, 1);
  }
};

}

#endif //DRAKE_ROLLPITCHYAWFLOATING_H
