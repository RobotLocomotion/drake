//
// Created by Twan Koolen on 4/27/16.
//

#ifndef DRAKE_JOINTTYPES_H
#define DRAKE_JOINTTYPES_H

namespace Drake {
template <typename J>
class JointType {
 public:
  virtual ~JointType() {};
};

template <typename J>
class QuaternionFloating : public JointType<J> {
 public:
  static const int NUM_POSITIONS = 7;
  static const int NUM_VELOCITIES = 6;

  template <typename DerivedQ>
  Transform3D<Promote<J, typename DerivedQ::Scalar>> jointTransform(const Eigen::MatrixBase<DerivedQ> &q) const {
    using T = Promote<J, typename DerivedQ::Scalar>;
    Transform3D<T> ret;
    ret.linear() = ConvertMatrix<T>()(quat2rotmat(q.template bottomRows<4>()));
    ret.translation() << Convert<T>()(q[0]), Convert<T>()(q[1]), Convert<T>()(q[2]);
    ret.makeAffine();
    return ret;
  }

  template <typename Q>
  MotionSubspace<Promote<J, Q>> motionSubspace() {
    return MotionSubspace<Promote<J, Q>>::Identity(TWIST_SIZE, QuaternionFloating::NUM_VELOCITIES);
  }

  template <typename Q>
  SpatialVector<Promote<J, Q>> motionSubspaceDotTimesV() {
    return SpatialVector<Promote<J, Q>>::Zero();
  };

  template <typename DerivedQ>
  ConfigurationDerivativeToVelocity<Promote<J, typename DerivedQ::Scalar>> configurationDerivativeToVelocity(const Eigen::MatrixBase<DerivedQ>& q) {
    using Q = typename DerivedQ::Scalar;
    using T = Promote<J, Q>;

    ConfigurationDerivativeToVelocity<T> ret(QuaternionFloating<J>::NUM_VELOCITIES, QuaternionFloating<J>::NUM_POSITIONS);
    auto quat = ConvertMatrix<T>()(q.template middleRows<QUAT_SIZE>(SPACE_DIMENSION));
    auto R = quat2rotmat(quat);

    Eigen::Matrix<T, 4, 1> quattilde;
    typename Gradient<Eigen::Matrix<T, 4, 1>, QUAT_SIZE, 1>::type dquattildedquat;
    normalizeVec(quat, quattilde, &dquattildedquat);
    auto RTransposeM = (R.transpose() * quatdot2angularvelMatrix(quat)).eval();
    ret.template block<3, 3>(0, 0).setZero();
    ret.template block<3, 4>(0, 3).noalias() = RTransposeM * dquattildedquat;
    ret.template block<3, 3>(3, 0) = R.transpose();
    ret.template block<3, 4>(3, 3).setZero();
    return ret;
  }

  template <typename DerivedQ>
  VelocityToConfigurationDerivative<Promote<J, typename DerivedQ::Scalar>> velocityToConfigurationDerivative(const Eigen::MatrixBase<DerivedQ>& q) {
    using Q = typename DerivedQ::Scalar;
    using T = Promote<J, Q>;

    VelocityToConfigurationDerivative<T> ret(QuaternionFloating<J>::NUM_POSITIONS, QuaternionFloating<J>::NUM_VELOCITIES);
    auto quat = ConvertMatrix<T>()(q.template middleRows<QUAT_SIZE>(SPACE_DIMENSION));
    auto R = quat2rotmat(quat);

    Eigen::Matrix<T, QUAT_SIZE, SPACE_DIMENSION> M;
    typename Gradient<decltype(M), QUAT_SIZE, 1>::type* dM = nullptr;
    angularvel2quatdotMatrix(quat, M, dM);

    ret.template block<3, 3>(0, 0).setZero();
    ret.template block<3, 3>(0, 3) = R;
    ret.template block<4, 3>(3, 0).noalias() = M * R;
    ret.template block<4, 3>(3, 3).setZero();
    return ret;
  }

  template <typename V>
  Eigen::Matrix<Promote<J, V>, Eigen::Dynamic, 1> frictionTorque() {
    return Eigen::Matrix<V, Eigen::Dynamic, 1>::Zero(QuaternionFloating<J>::NUM_VELOCITIES, 1);
  }
};

}


#endif //DRAKE_JOINTTYPES_H
