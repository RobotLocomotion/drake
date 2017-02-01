#pragma once

#include <memory>
#include <string>

#include "drake/common/eigen_types.h"
#include "drake/multibody/joints/fixed_axis_one_dof_joint.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
class HelicalJoint : public FixedAxisOneDoFJoint<HelicalJoint> {
 public:
  HelicalJoint(const std::string& name,
               const Eigen::Isometry3d& transform_to_parent_body,
               const Eigen::Vector3d& axis, double pitch)
      : FixedAxisOneDoFJoint<HelicalJoint>(*this, name,
                                           transform_to_parent_body,
                                           spatialJointAxis(axis, pitch)),
        axis_(axis),
        pitch_(pitch) {}

  virtual ~HelicalJoint() {}

  std::unique_ptr<DrakeJoint> Clone() const final;

  template <typename DerivedQ>
  Eigen::Transform<typename DerivedQ::Scalar, 3, Eigen::Isometry>
  jointTransform(const Eigen::MatrixBase<DerivedQ>& q) const {
    typedef typename DerivedQ::Scalar Scalar;
    Eigen::Transform<Scalar, 3, Eigen::Isometry> ret;
    ret = Eigen::AngleAxis<Scalar>(q[0], axis_.cast<Scalar>()) *
          Eigen::Translation<Scalar, 3>(q[0] * (pitch_ * axis_).cast<Scalar>());
    ret.makeAffine();
    return ret;
  }

  bool CompareToClone(const DrakeJoint& other) const final;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  static drake::TwistVector<double> spatialJointAxis(
      const Eigen::Vector3d& axis, double pitch);

  const Eigen::Vector3d axis_;
  const double pitch_;
};
#pragma GCC diagnostic pop  // pop -Wno-overloaded-virtual
