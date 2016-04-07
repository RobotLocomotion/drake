#ifndef DRAKE_SYSTEMS_PLANTS_JOINTS_HELICALJOINT_H_
#define DRAKE_SYSTEMS_PLANTS_JOINTS_HELICALJOINT_H_

#include "FixedAxisOneDoFJoint.h"

class DRAKEJOINTS_EXPORT HelicalJoint
    : public FixedAxisOneDoFJoint<HelicalJoint> {
  // disable copy construction and assignment
  // HelicalJoint(const HelicalJoint&) = delete;
  // HelicalJoint& operator=(const HelicalJoint&) = delete;

 private:
  const Eigen::Vector3d axis;
  const double pitch;

 public:
  HelicalJoint(const std::string& name,
               const Eigen::Isometry3d& transform_to_parent_body,
               const Eigen::Vector3d& axis, double pitch)
      : FixedAxisOneDoFJoint<HelicalJoint>(*this, name,
                                           transform_to_parent_body,
                                           spatialJointAxis(axis, pitch)),
        axis(axis),
        pitch(pitch) {}

  virtual ~HelicalJoint() {}

  template <typename DerivedQ>
  Eigen::Transform<typename DerivedQ::Scalar, 3, Eigen::Isometry>
  jointTransform(const Eigen::MatrixBase<DerivedQ>& q) const {
    typedef typename DerivedQ::Scalar Scalar;
    Eigen::Transform<Scalar, 3, Eigen::Isometry> ret;
    ret = Eigen::AngleAxis<Scalar>(q[0], axis.cast<Scalar>()) *
          Eigen::Translation<Scalar, 3>(q[0] * (pitch * axis).cast<Scalar>());
    ret.makeAffine();
    return ret;
  }

 private:
  static Eigen::Matrix<double, TWIST_SIZE, 1> spatialJointAxis(
      const Eigen::Vector3d& axis, double pitch);

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif  // DRAKE_SYSTEMS_PLANTS_JOINTS_HELICALJOINT_H_
