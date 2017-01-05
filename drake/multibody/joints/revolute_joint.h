#pragma once

#include <string>

#include <Eigen/Geometry>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/joints/fixed_axis_one_dof_joint.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
class RevoluteJoint : public FixedAxisOneDoFJoint<RevoluteJoint> {
  // disable copy construction and assignment
  // RevoluteJoint(const RevoluteJoint&) = delete;
  // RevoluteJoint& operator=(const RevoluteJoint&) = delete;

 private:
  // Rotation axis expressed in the joint's inboard frame F.
  Eigen::Vector3d rotation_axis;

 public:
  /// RevoluteJoint constructor.
  /// Pose and axis are measured and expressed in the
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
  RevoluteJoint(const std::string& name,
                const Eigen::Isometry3d& X_PF,
                const Eigen::Vector3d& rotation_axis_F)
      : FixedAxisOneDoFJoint<RevoluteJoint>(*this, name,
                                            X_PF,
                                            spatialJointAxis(rotation_axis_F)),
        rotation_axis(rotation_axis_F) {
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
#pragma GCC diagnostic pop  // pop -Wno-overloaded-virtual
