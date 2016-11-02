#pragma once

#include <string>

#include <Eigen/Geometry>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/plants/joints/FixedAxisOneDoFJoint.h"

class DRAKE_EXPORT RevoluteJoint
    : public FixedAxisOneDoFJoint<RevoluteJoint> {
  // disable copy construction and assignment
  // RevoluteJoint(const RevoluteJoint&) = delete;
  // RevoluteJoint& operator=(const RevoluteJoint&) = delete;

 private:
  Eigen::Vector3d rotation_axis;

 public:
  RevoluteJoint(const std::string& name,
                const Eigen::Isometry3d& transform_to_parent_body,
                const Eigen::Vector3d& _rotation_axis)
      : FixedAxisOneDoFJoint<RevoluteJoint>(*this, name,
                                            transform_to_parent_body,
                                            spatialJointAxis(_rotation_axis)),
        rotation_axis(_rotation_axis) {
    DRAKE_ASSERT(std::abs(rotation_axis.norm() - 1.0) < 1e-10);
  }

  virtual ~RevoluteJoint() {}

  template <typename DerivedQ>
  Eigen::Transform<typename DerivedQ::Scalar, 3, Eigen::Isometry>
  jointTransform(const Eigen::MatrixBase<DerivedQ>& q) const {
    typedef typename DerivedQ::Scalar Scalar;
    Eigen::Transform<Scalar, 3, Eigen::Isometry> ret(
        Eigen::AngleAxis<Scalar>(q[0], rotation_axis.cast<Scalar>()));
    ret.makeAffine();
    return ret;
  }

 private:
  static drake::TwistVector<double> spatialJointAxis(
      const Eigen::Vector3d& rotation_axis);

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
