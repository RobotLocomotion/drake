#ifndef HELICALJOINT_H_
#define HELICALJOINT_H_

#include "FixedAxisOneDoFJoint.h"

class HelicalJoint: public FixedAxisOneDoFJoint
{
  // disable copy construction and assignment
  HelicalJoint(const HelicalJoint&) = delete;
  HelicalJoint& operator=(const HelicalJoint&) = delete;

private:
  const Eigen::Vector3d axis;
  const double pitch;

public:
  HelicalJoint(const std::string& name, const Eigen::Isometry3d& transform_to_parent_body, const Eigen::Vector3d& axis, double pitch);

  virtual ~HelicalJoint();

  virtual Eigen::Isometry3d jointTransform(double* const q) const; //override;

private:
  static Eigen::Matrix<double, TWIST_SIZE, 1> spatialJointAxis(const Eigen::Vector3d& axis, double pitch);
};

#endif /* HELICALJOINT_H_ */
