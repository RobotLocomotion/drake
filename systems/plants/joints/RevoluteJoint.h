#ifndef REVOLUTEJOINT_H_
#define REVOLUTEJOINT_H_

#include "FixedAxisOneDoFJoint.h"

class RevoluteJoint: public FixedAxisOneDoFJoint
{
private:
  e::Vector3d rotation_axis;
public:
  RevoluteJoint(const RigidBody& parent_body, const e::AffineCompact3d& transform_to_parent_body, const e::Vector3d& rotation_axis);

  virtual ~RevoluteJoint();

  virtual e::AffineCompact3d jointTransform(double* const q) const override;

private:
  static e::Matrix<double, TWIST_SIZE, 1> spatialJointAxis(const e::Vector3d& rotation_axis);
};

#endif /* REVOLUTEJOINT_H_ */
