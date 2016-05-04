#ifndef DRAKE_MULTIBODY_DYNAMICS_JOINTS_REVOLUTEJOINT_H_H
#define DRAKE_MULTIBODY_DYNAMICS_JOINTS_REVOLUTEJOINT_H_H

#include "drake/multibody_dynamics/joints/FixedAxisOneDofJoint.h"

namespace drake {

template <typename Scalar>
class RevoluteJoint : public FixedAxisOneDoFJoint<Scalar> {
 public:
  using FixedAxisOneDoFJoint<Scalar>::getJointAxis;

  RevoluteJoint(const std::string& name, const Transform3D<Scalar> &transform_to_parent_body, const Eigen::Matrix<Scalar, 3, 1>& rotation_axis) :
      FixedAxisOneDoFJoint<Scalar>(name, transform_to_parent_body, createSpatialAxis(rotation_axis)) {
    // empty
  }

  virtual Transform3D<Scalar> jointTransform(const Eigen::Ref<VectorX<Scalar>> &q) const override {
    const auto& joint_axis = getJointAxis();
    Transform3D<Scalar> ret(Eigen::AngleAxis<Scalar>(q[0], joint_axis.template topRows<3>()));
    ret.makeAffine();
    return ret;
  }

 private:
  static SpatialVector<Scalar> createSpatialAxis(const Eigen::Matrix<Scalar, 3, 1>& rotation_axis) {
    auto ret = SpatialVector<Scalar>::Zero().eval();
    ret.template topRows<3>() = rotation_axis;
    return ret;
  }
};

}

#endif //DRAKE_MULTIBODY_DYNAMICS_JOINTS_REVOLUTEJOINT_H_H
