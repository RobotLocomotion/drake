#pragma once

#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/joints/fixed_axis_one_dof_joint.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
class RevoluteJoint : public FixedAxisOneDoFJoint<RevoluteJoint> {
 public:
  RevoluteJoint(const std::string& name,
                const Eigen::Isometry3d& transform_to_parent_body,
                const Eigen::Vector3d& rotation_axis)
      : FixedAxisOneDoFJoint<RevoluteJoint>(*this, name,
                                            transform_to_parent_body,
                                            spatialJointAxis(rotation_axis)),
        rotation_axis_(rotation_axis) {
    DRAKE_ASSERT(std::abs(rotation_axis_.norm() - 1.0) < 1e-10);
  }

  virtual ~RevoluteJoint() {}

  template <typename DerivedQ>
  Eigen::Transform<typename DerivedQ::Scalar, 3, Eigen::Isometry>
  jointTransform(const Eigen::MatrixBase<DerivedQ>& q) const {
    typedef typename DerivedQ::Scalar Scalar;
    Eigen::Transform<Scalar, 3, Eigen::Isometry> ret(
        Eigen::AngleAxis<Scalar>(q[0], rotation_axis_.cast<Scalar>()));
    ret.makeAffine();
    return ret;
  }

  const Eigen::Vector3d& rotation_axis() const { return rotation_axis_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  std::unique_ptr<DrakeJoint> DoClone() const final;

 private:
  static drake::TwistVector<double> spatialJointAxis(
      const Eigen::Vector3d& rotation_axis);

  Eigen::Vector3d rotation_axis_;
};
#pragma GCC diagnostic pop  // pop -Wno-overloaded-virtual
