#ifndef DRAKE_MULTIBODY_DYNAMICS_JOINTS_PRISMATICJOINT_H
#define DRAKE_MULTIBODY_DYNAMICS_JOINTS_PRISMATICJOINT_H

#include "drake/multibody_dynamics/joints/FixedAxisOneDoFJoint.h"

namespace drake {

template <typename Scalar>
class PrismaticJoint : public FixedAxisOneDoFJoint<Scalar> {
 public:
  using FixedAxisOneDoFJoint<Scalar>::GetJointAxis;

  PrismaticJoint(const std::string& name, const Transform3D<Scalar> &transform_to_parent_body, const Eigen::Matrix<Scalar, 3, 1>& translation_axis) :
      FixedAxisOneDoFJoint<Scalar>(name, transform_to_parent_body, CreateSpatialAxis(translation_axis)) {
    // empty
  }

  virtual Transform3D<Scalar> JointTransform(const Eigen::Ref<VectorX<Scalar>> &q) const override {
    Transform3D<Scalar> ret;
    ret.linear().setIdentity();
    const auto& joint_axis = GetJointAxis();
    ret.translation() = q[0] * joint_axis.template bottomRows<3>();
    ret.makeAffine();
    return ret;
  }

    private:
  static SpatialVector<Scalar> CreateSpatialAxis(const Eigen::Matrix<Scalar, 3, 1>& translation_axis) {
    auto ret = SpatialVector<Scalar>::Zero().eval();
    ret.template bottomRows<3>() = translation_axis;
    return ret;
  }
};

}

#endif //DRAKE_MULTIBODY_DYNAMICS_JOINTS_PRISMATICJOINT_H
