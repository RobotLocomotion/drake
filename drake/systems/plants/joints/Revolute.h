#ifndef DRAKE_REVOLUTE_H
#define DRAKE_REVOLUTE_H

#include "drake/systems/plants/joints/FixedAxisOneDoF.h"

namespace Drake {

template <typename J>
class Revolute : public FixedAxisOneDoF<J> {
 public:
  using FixedAxisOneDoF<J>::joint_axis;

  Revolute(const Eigen::Matrix<J, 3, 1>& rotation_axis) : FixedAxisOneDoF<J>(createSpatialAxis(rotation_axis)) { }

  template <typename DerivedQ>
  Transform3D<Promote<J, typename DerivedQ::Scalar>> jointTransform(const Eigen::MatrixBase<DerivedQ>& q) const {
    using Q = typename DerivedQ::Scalar;
    using T = Promote<J, Q>;

    Transform3D<T> ret(Eigen::AngleAxis<T>(Convert<T>()(q[0]), ConvertMatrix<T>()(joint_axis.template topRows<3>())));
    ret.makeAffine();
    return ret;
  }

 private:
  static inline SpatialVector<J> createSpatialAxis(const Eigen::Matrix<J, 3, 1>& rotation_axis) {
    auto ret = SpatialVector<J>::Zero().eval();
    ret.template topRows<3>() = rotation_axis;
    return ret;
  }

};

}

#endif //DRAKE_REVOLUTE_H
