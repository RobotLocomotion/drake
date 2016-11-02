#pragma once

#include <string>

#include "drake/common/eigen_types.h"
#include "drake/systems/plants/joints/FixedAxisOneDoFJoint.h"

class DRAKE_EXPORT HelicalJoint
    : public FixedAxisOneDoFJoint<HelicalJoint> {
  // disable copy construction and assignment
  // HelicalJoint(const HelicalJoint&) = delete;
  // HelicalJoint& operator=(const HelicalJoint&) = delete;

 private:
  const Eigen::Vector3d axis_;
  const double pitch_;

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

 private:
  static drake::TwistVector<double> spatialJointAxis(
      const Eigen::Vector3d& axis, double pitch);

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
