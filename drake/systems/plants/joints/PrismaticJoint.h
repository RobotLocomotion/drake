#ifndef PRISMATICJOINT_H_
#define PRISMATICJOINT_H_

#include "FixedAxisOneDoFJoint.h"

class DLLEXPORT_DRAKEJOINT PrismaticJoint: public FixedAxisOneDoFJoint<PrismaticJoint>
{
  // disable copy construction and assignment
  // not available in MSVC2010...
  // PrismaticJoint(const PrismaticJoint&) = delete;
  // PrismaticJoint& operator=(const PrismaticJoint&) = delete;

private:
  Eigen::Vector3d translation_axis;
public:

  PrismaticJoint(const std::string& name, const Eigen::Isometry3d& transform_to_parent_body, const Eigen::Vector3d& translation_axis) :
      FixedAxisOneDoFJoint<PrismaticJoint>(*this, name, transform_to_parent_body, spatialJointAxis(translation_axis)), translation_axis(translation_axis) {
    assert(std::abs(translation_axis.norm() - 1.0) < 1e-10);
  };

  template<typename DerivedQ>
  Eigen::Transform<typename DerivedQ::Scalar, 3, Eigen::Isometry> jointTransform(const Eigen::MatrixBase<DerivedQ> &q) const {
    typedef typename DerivedQ::Scalar Scalar;
    Eigen::Transform<Scalar, 3, Eigen::Isometry> ret;
    ret.linear().setIdentity();
    ret.translation() = q[0] * translation_axis.cast<Scalar>();
    ret.makeAffine();
    return ret;
  }

  virtual ~PrismaticJoint() { } ;

private:
  static Eigen::Matrix<double, TWIST_SIZE, 1> spatialJointAxis(const Eigen::Vector3d& translation_axis);
};

#endif /* PRISMATICJOINT_H_ */
