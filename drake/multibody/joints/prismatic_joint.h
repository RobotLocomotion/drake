#pragma once

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
  // disable copy construction and assignment
  // PrismaticJoint(const PrismaticJoint&) = delete;
  // PrismaticJoint& operator=(const PrismaticJoint&) = delete;

 public:
  /// The constructor that intializes the name, position, and axis of motion
  /// of this prismatic joint. Pose and axis are measured and expressed in the
  /// parent body frame P. For more information on frames see
  /// @ref rigid_body_tree_frames.
  ///
  /// @param[in] name The name of this joint.
  ///
  /// @param[in] X_PF The pose of the inboard frame F measured and expressed
  /// in the parent body frame P.
  ///
  /// @param[in] translation_axis_F Translation axis expressed in the joint's
  /// inboard frame F.
  PrismaticJoint(const std::string& name,
                 const Eigen::Isometry3d& X_PF,
                 const Eigen::Vector3d& translation_axis_F)
      : FixedAxisOneDoFJoint<PrismaticJoint>(
            *this, name, X_PF,
            spatialJointAxis(translation_axis_F)),
        translation_axis_(translation_axis_F) {
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
  // Translation axis expressed in the joint's inboard frame F.
  Eigen::Vector3d translation_axis_;
};
#pragma GCC diagnostic pop  // pop -Wno-overloaded-virtual
