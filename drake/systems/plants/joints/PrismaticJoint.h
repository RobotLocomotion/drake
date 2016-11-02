#pragma once

#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/plants/joints/FixedAxisOneDoFJoint.h"

/**
 * A prismatic joint moves linearly along one axis.
 */
class DRAKE_EXPORT PrismaticJoint
    : public FixedAxisOneDoFJoint<PrismaticJoint> {
  // disable copy construction and assignment
  // PrismaticJoint(const PrismaticJoint&) = delete;
  // PrismaticJoint& operator=(const PrismaticJoint&) = delete;

 public:
  /**
   * The constructor that intializes the name, position, and axis of motion
   * of this prismatic joint.
   *
   * @param[in] name The name of this joint.
   *
   * @param[in] transform_to_parent_body The transform from this joint's frame
   * to this joint's parent's frame.
   *
   * @param[in] translation_axis The axis along which this joint moves.
   */
  PrismaticJoint(const std::string& name,
                 const Eigen::Isometry3d& transform_to_parent_body,
                 const Eigen::Vector3d& translation_axis)
      : FixedAxisOneDoFJoint<PrismaticJoint>(
            *this, name, transform_to_parent_body,
            spatialJointAxis(translation_axis)),
        translation_axis_(translation_axis) {
    DRAKE_ASSERT(std::abs(translation_axis_.norm() - 1.0) < 1e-10);
  }

  template <typename DerivedQ>
  Eigen::Transform<typename DerivedQ::Scalar, 3, Eigen::Isometry>
  jointTransform(const Eigen::MatrixBase<DerivedQ>& q) const {
    typedef typename DerivedQ::Scalar Scalar;
    Eigen::Transform<Scalar, 3, Eigen::Isometry> ret;
    ret.linear().setIdentity();
    ret.translation() = q[0] * translation_axis_.cast<Scalar>();
    ret.makeAffine();
    return ret;
  }

  virtual ~PrismaticJoint() {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  static drake::TwistVector<double> spatialJointAxis(
      const Eigen::Vector3d& translation_axis);

 private:
  Eigen::Vector3d translation_axis_;
};
