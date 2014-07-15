#ifndef HELICALJOINT_H_
#define HELICALJOINT_H_

#include "FixedAxisOneDoFJoint.h"

class HelicalJoint: public FixedAxisOneDoFJoint
{
private:
  const e::Vector3d axis;
  const double pitch;

public:
  HelicalJoint(const RigidBody& parent_body, const e::AffineCompact3d& transform_to_parent_body, const e::Vector3d& axis, double pitch);

  virtual ~HelicalJoint();

  virtual e::AffineCompact3d jointTransform(double* const q) const override;

private:
  static e::Matrix<double, TWIST_SIZE, 1> spatialJointAxis(const e::Vector3d& axis, double pitch);
};

#endif /* HELICALJOINT_H_ */
