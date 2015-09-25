#ifndef REVOLUTEJOINT_H_
#define REVOLUTEJOINT_H_

#include "FixedAxisOneDoFJoint.h"
#include <Eigen/Geometry>

class DLLEXPORT_DRAKEJOINT RevoluteJoint: public FixedAxisOneDoFJoint<RevoluteJoint>
{
  // disable copy construction and assignment
  // not available in MSVC2010...
  // RevoluteJoint(const RevoluteJoint&) = delete;
  // RevoluteJoint& operator=(const RevoluteJoint&) = delete;

private:
  Eigen::Vector3d rotation_axis;
public:
  RevoluteJoint(const std::string& name, const Eigen::Isometry3d& transform_to_parent_body, const Eigen::Vector3d& rotation_axis) :
      FixedAxisOneDoFJoint(*this, name, transform_to_parent_body, spatialJointAxis(rotation_axis)), rotation_axis(rotation_axis)
  {
    assert(std::abs(rotation_axis.norm() - 1.0) < 1e-10);
  };

  virtual ~RevoluteJoint() {};

  template<typename DerivedQ>
  Eigen::Transform<typename DerivedQ::Scalar, 3, Eigen::Isometry> jointTransform(const Eigen::MatrixBase<DerivedQ> &q) const {
    typedef typename DerivedQ::Scalar Scalar;
    Eigen::Transform<Scalar, 3, Eigen::Isometry> ret(Eigen::AngleAxis<Scalar>(q[0], rotation_axis.cast<Scalar>()));
    ret.makeAffine();
    return ret;
  }

private:
  static Eigen::Matrix<double, TWIST_SIZE, 1> spatialJointAxis(const Eigen::Vector3d& rotation_axis);
};

#endif /* REVOLUTEJOINT_H_ */
