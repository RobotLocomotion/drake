#ifndef DRAKE_MULTIBODY_DYNAMICS_JOINTS_HELICALJOINT_H
#define DRAKE_MULTIBODY_DYNAMICS_JOINTS_HELICALJOINT_H

#include "drake/multibody_dynamics/joints/FixedAxisOneDoFJoint.h"

namespace drake {

template <typename Scalar>
class HelicalJoint : public FixedAxisOneDoFJoint<Scalar> {
 public:
  using FixedAxisOneDoFJoint<Scalar>::getJointAxis;

  HelicalJoint(const std::string& name, const Transform3D<Scalar> &transform_to_parent_body, const Eigen::Matrix<Scalar, 3, 1>& axis, const Scalar& pitch) :
      FixedAxisOneDoFJoint<Scalar>(name, transform_to_parent_body, createSpatialAxis(axis, pitch)) {
    // empty
  }

  virtual Transform3D<Scalar> jointTransform(const Eigen::Ref<VectorX<Scalar>> &q) const override {
    const auto& joint_axis = getJointAxis();
    auto rotation = Eigen::AngleAxis<Scalar>(q[0], joint_axis.template topRows<3>());
    auto translation = Eigen::Translation<Scalar, 3>(q[0] * joint_axis.template bottomRows<3>());
    Transform3D<Scalar> ret(rotation * translation);
    ret.makeAffine();
    return ret;
  }

 private:
  static SpatialVector<Scalar> createSpatialAxis(const Eigen::Matrix<Scalar, 3, 1>& axis, const Scalar& pitch) {
    auto ret = SpatialVector<Scalar>();
    ret.template topRows<3>() = axis;
    ret.template bottomRows<3>() = pitch * axis;
    return ret;
  }
};

}

#endif //DRAKE_MULTIBODY_DYNAMICS_JOINTS_HELICALJOINT_H
