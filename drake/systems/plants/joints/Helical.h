#ifndef DRAKE_HELICAL_H
#define DRAKE_HELICAL_H


#include "drake/systems/plants/joints/FixedAxisOneDoF.h"

namespace Drake {

template <typename J>
class Helical : public FixedAxisOneDoF<J> {
 public:
  using FixedAxisOneDoF<J>::joint_axis;

  Helical(const Eigen::Matrix<J, 3, 1>& axis, const J& pitch) : FixedAxisOneDoF<J>(createSpatialAxis(axis, pitch)) { }

  template <typename DerivedQ>
  Transform3D<Promote<J, typename DerivedQ::Scalar>> jointTransform(const Eigen::MatrixBase<DerivedQ>& q) const {
    using Q = typename DerivedQ::Scalar;
    using T = Promote<J, Q>;

    auto rotation = Eigen::AngleAxis<T>(Convert<T>()(q[0]), ConvertMatrix<T>()(joint_axis.template topRows<3>()));
    auto translation = Eigen::Translation<T, 3>(Convert<T>()(q[0]) * ConvertMatrix<T>()(joint_axis.template bottomRows<3>()));
    Transform3D<T> ret(rotation * translation);
    ret.makeAffine();
    return ret;
  }

 private:
  static inline SpatialVector<J> createSpatialAxis(const Eigen::Matrix<J, 3, 1>& axis, const J& pitch) {
    auto ret = SpatialVector<J>();
    ret.template topRows<3>() = axis;
    ret.template bottomRows<3>() = pitch * axis;
    return ret;
  }
};

}

#endif //DRAKE_HELICAL_H
