#ifndef REVOLUTEJOINT_H_
#define REVOLUTEJOINT_H_

#include "FixedAxisOneDoFJoint.h"

class DLLEXPORT_DRAKEJOINT RevoluteJoint: public FixedAxisOneDoFJoint
{
  // disable copy construction and assignment
  // not available in MSVC2010...
  // RevoluteJoint(const RevoluteJoint&) = delete;
  // RevoluteJoint& operator=(const RevoluteJoint&) = delete;

private:
  Eigen::Vector3d rotation_axis;
public:
  RevoluteJoint(const std::string& name, const Eigen::Isometry3d& transform_to_parent_body, const Eigen::Vector3d& rotation_axis);

  virtual ~RevoluteJoint();

  virtual Eigen::Isometry3d jointTransform(const Eigen::Ref<const Eigen::VectorXd>& q) const; //override;

private:
  static Eigen::Matrix<double, TWIST_SIZE, 1> spatialJointAxis(const Eigen::Vector3d& rotation_axis);
};

#endif /* REVOLUTEJOINT_H_ */
