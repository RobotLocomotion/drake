#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/joints/fixed_axis_one_dof_joint.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
/**
 * A prismatic joint moves linearly along one axis.
 */
class PrismaticJoint : public FixedAxisOneDoFJoint<PrismaticJoint> {
 public:
  /**
   * The constructor that initializes the name, position, and axis of motion
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

  const Eigen::Vector3d& translation_axis() const { return translation_axis_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  std::unique_ptr<DrakeJoint> DoClone() const final;

 private:
  static drake::TwistVector<double> spatialJointAxis(
      const Eigen::Vector3d& translation_axis);

  Eigen::Vector3d translation_axis_;
};
#pragma GCC diagnostic pop  // pop -Wno-overloaded-virtual
